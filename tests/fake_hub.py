#!/usr/bin/env python3
"""
Software stand-in for an SV241 Pro (firmware V1.3 behaviour as documented in docs/PROTOCOL.md),
for exercising sv241.py without hardware. No pyserial needed.

    python tests/fake_hub.py tui            # any sv241.py command line, against the simulator
    python tests/fake_hub.py status --json

Modelled: 6-byte frames + sum%255 checksum (frames with a bad checksum are dropped), cmds 01 set,
03-07 sensors (base-255, +255 offsets on temps/RH), 08 state, FE/FF all on/off, and the PWM1 dew
cutoff (lens above limit -> PWM1 forced to 0, writes refused, previous level restored on release).
The lens probe warms (with lag) toward ~PWM1 duty, so 100 % trips the cutoff and ~75 % does not. Not modelled: timing, banner.

Simplifications vs real firmware (all invisible to sv241.py, which never relies on them):
  - CMD 08 reports outputs as stored (00/FF); the firmware reports 0/1. sv241 tests "!= 0".
  - CMD 01 to an output accepts any non-zero value as on; the firmware refuses values other
    than 00/FF. sv241 only ever sends 00/FF.
  - FE/FF acks carry payload "FE 00"/"FF 00"; the firmware sends "FE FF"/"FF FF". sv241 ignores
    that ack and verifies by read-back.
  - The cutoff is re-evaluated on every request rather than every 5 s; no "mode" bookkeeping.
  - Exactly one 6-byte frame per write() call (the pty test server slices its stream to match).

Structure: FakeSerial is the device (pyserial-like surface: write/read/reset_input_buffer/
flush/close/in_waiting); FakeHub is a sv241.Hub whose open() attaches a shared FakeSerial
(class attribute `sim`, so tests can reset and inspect it) instead of a real port.
"""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import sv241  # noqa: E402


def enc255(x):
    """Encode a sensor value as the 4 payload bytes: x*100 in base 255, most significant first
    (the inverse of sv241.base255; matches the firmware for every value the sim produces)."""
    x = int(round(x * 100))
    out = []
    for _ in range(4):
        out.append(x % 255)
        x //= 255
    return bytes(reversed(out))


def reply(cmd, payload):
    """Build a reply frame 24 len cmd payload... chk (len = payload + 4, chk = sum % 255)."""
    b = bytes([0x24, len(payload) + 4, cmd]) + bytes(payload)
    return b + bytes([sum(b) % 255])


class FakeSerial:
    """Simulated hub behind a serial port. Requests are answered synchronously: write() appends
    the reply to rx, read() drains it.

    State: outs (7 output bytes, 00/FF), ch (set index -> raw: 7 ADJ, 8 PWM1, 9 PWM2; starts at
    the values typically seen on the real hub: ADJ 198, PWM 253), cutoff flag and the PWM1 level
    saved at cutoff entry. Sensors: fixed input voltage, ambient and RH; the lens follows PWM1
    (see lens()); current rises with outputs on and PWM duty. `log` records every request."""

    def __init__(self, ambient=8.0, rh=70.0, lens_gain=4.5, noise=0.05, seed=1):
        """ambient (°C) and rh (%) set the cutoff limit; lens_gain = lens rise (°C) at full PWM1;
        noise = uniform jitter on the lens reading; seed makes runs repeatable."""
        self.outs = [0xFF] * 7
        self.ch = {7: 198, 8: 253, 9: 253}
        self.ambient, self.rh, self.lens_gain, self.noise = ambient, rh, lens_gain, noise
        self.rng = random.Random(seed)
        self.cutoff = False
        self.lens_t = ambient + 1.0
        self.saved_pwm1 = None
        self.rx = bytearray()
        self.log = []          # every request frame received

    # pyserial surface used by sv241.Hub
    in_waiting = 0

    def read(self, n):
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def reset_input_buffer(self):
        self.rx.clear()

    def flush(self):
        pass

    def close(self):
        pass

    def lens(self):
        """First-order lag toward ambient + 1 + gain * PWM1 duty (advances once per request)."""
        target = self.ambient + 1.0 + self.lens_gain * self.ch[8] / 255.0
        self.lens_t += 0.05 * (target - self.lens_t)
        return self.lens_t + self.rng.uniform(-self.noise, self.noise)

    def update_cutoff(self):
        """Firmware dew cutoff: lens above limit -> save PWM1, force 0, refuse writes; back under
        -> restore the saved level. No hysteresis, like the firmware."""
        lim = sv241.cutoff_limit(self.ambient, self.rh)
        hot = self.lens() > lim
        if hot and not self.cutoff:
            self.cutoff, self.saved_pwm1, self.ch[8] = True, self.ch[8], 0
        elif not hot and self.cutoff:
            self.cutoff, self.ch[8] = False, self.saved_pwm1

    def write(self, b):
        """Process one request frame and queue its reply (none for a bad frame or unknown cmd)."""
        b = bytes(b)
        self.log.append(b)
        if len(b) != 6 or b[0] != 0x24 or sum(b[:5]) % 255 != b[5]:
            return                                   # firmware drops bad frames silently
        cmd, idx, val = b[2], b[3], b[4]
        self.update_cutoff()
        if cmd == 0x08:
            self.rx += reply(8, self.outs + [self.ch[7], self.ch[8], self.ch[9]])
        elif cmd == 0x01:
            ok = True
            if idx <= 6:
                self.outs[idx] = 0xFF if val else 0x00
            elif idx in self.ch:
                if idx == 8 and self.cutoff:
                    ok = False                       # refused: level untouched
                else:
                    self.ch[idx] = val
            else:
                ok = False
            self.rx += reply(1, [idx, 0xFF if ok else 0x00])
        elif cmd == 0xFE:
            self.outs = [0xFF] * 7
            self.rx += reply(0xFE, [0xFE, 0x00])
        elif cmd == 0xFF:
            self.outs = [0x00] * 7
            self.rx += reply(0xFF, [0xFF, 0x00])
        elif cmd in (3, 4, 5, 6, 7):
            on = sum(1 for o in self.outs if o)
            amps = 0.15 + 0.25 * on + 1.6 * (self.ch[8] + self.ch[9]) / 510.0
            val = {3: 12.4, 4: self.lens() + 255, 5: self.ambient + 255, 6: self.rh + 255,
                   7: amps * 1000 / sv241.INA_SCALE}[cmd]
            self.rx += reply(cmd, enc255(val))


class FakeHub(sv241.Hub):
    """sv241.Hub wired to the simulator: pass as main(..., hub_factory=FakeHub). Skips port
    discovery and the open-time settle delay and banner scan; xfer(), reads and verified writes
    are the real Hub code. The FakeSerial instance is shared (FakeHub.sim) and created on first open."""
    sim = None

    def open(self):
        if self.ser:
            return self
        if FakeHub.sim is None:
            FakeHub.sim = FakeSerial()
        self.port = "SIM"
        self.ser = FakeHub.sim
        return self


if __name__ == "__main__":
    sys.exit(sv241.main(sys.argv[1:], hub_factory=FakeHub))
