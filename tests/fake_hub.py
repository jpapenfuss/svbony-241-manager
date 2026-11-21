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
"""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import sv241  # noqa: E402


def enc255(x):
    x = int(round(x * 100))
    out = []
    for _ in range(4):
        out.append(x % 255)
        x //= 255
    return bytes(reversed(out))


def reply(cmd, payload):
    b = bytes([0x24, len(payload) + 4, cmd]) + bytes(payload)
    return b + bytes([sum(b) % 255])


class FakeSerial:
    def __init__(self, ambient=8.0, rh=70.0, lens_gain=4.5, noise=0.05, seed=1):
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
        lim = sv241.cutoff_limit(self.ambient, self.rh)
        hot = self.lens() > lim
        if hot and not self.cutoff:
            self.cutoff, self.saved_pwm1, self.ch[8] = True, self.ch[8], 0
        elif not hot and self.cutoff:
            self.cutoff, self.ch[8] = False, self.saved_pwm1

    def write(self, b):
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
