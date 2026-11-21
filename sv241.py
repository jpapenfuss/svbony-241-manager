#!/usr/bin/env python3
"""
sv241.py: control and monitor an SVBONY SV241 Pro power hub.

One-shot commands (open the port, do one thing, close it):

    python sv241.py status                 # outputs, PWM/ADJ levels, sensors
    python sv241.py status --json
    python sv241.py sensors [--json]
    python sv241.py on dc1 dc3 usbc        # switch outputs on (dc1..dc5, usbc, usba)
    python sv241.py off dc2
    python sv241.py on all                 # firmware all-on  (24 06 FE FE 00 28)
    python sv241.py off all                # firmware all-off (24 06 FF FF 00 2A)
    python sv241.py set pwm1 40            # dew heater 1 to 40 %
    python sv241.py set pwm2 raw:120       # raw 0..255
    python sv241.py set adj 12             # ADJ OUT to ~12 V (approximate scale, see below)
    python sv241.py raw 08                 # send cmd 0x08, print reply hex
    python sv241.py ports                  # list serial ports, CH340s flagged

Hold mode (long-running monitor; the default when no command is given, so the old
SVBONY-hold.py command lines still work unchanged):

    python sv241.py --port COM6 --pwm1 40 --pwm2 40 --adj 12
    python sv241.py hold --pwm1 raw:100
    python sv241.py --hold-current            # adopt the hub's current values as targets
    python sv241.py                           # monitor only, enforce nothing
    python sv241.py --pwm1 40 --csv hub.csv   # also log every poll to CSV
    python sv241.py --pwm1 40 --no-clear      # plain scrolling event log (for services)

Interactive mode (live display; switch outputs and change levels with the keyboard):

    python sv241.py --port COM6 tui
    python sv241.py tui --pwm1 75 --interval 1   # start with PWM1 held at 75 %

    Up/Down (j/k) select a row; Space/Enter toggles an output (off asks y/N); Left/Right or -/+
    step ADJ 0.5 V / PWM 5 %, ',' '.' step 1 raw (writes are sent 0.6 s after the last key);
    e or Enter on a channel types an exact value (12, 40, raw:120); h holds/releases a channel
    (re-pushed like hold mode); a all on; x all off (asks y/N); r refresh now; q quit.
    Pure stdlib: ANSI escapes plus msvcrt (Windows) or termios (POSIX); no curses needed.

Hold mode polls outputs, PWM/ADJ channels and sensors, shows them on a refreshing console display,
and re-pushes any target the hub no longer reports. Every push is read back to confirm it took.
Ctrl+C exits and leaves settings as they are.

Global options go before the command: --port COM6, --timeout 0.4.

One-shot exit codes: 0 success, 1 comm error, 2 hub did not take the value (e.g. PWM1 refused
by the firmware dew cutoff) or bad argument. Settings persist on the hub (it saves them to flash).

Only one program can hold the serial port: close NINA's connection to the hub, the vendor app,
or another running copy of this script first.

Why PWM1 drops to 0
-------------------
The firmware has a built-in "dew cutoff" on PWM1 only. Every 5 s it compares the lens probe
against a limit derived from ambient temperature and humidity:

    ambient > 15 °C      limit = dew point + 7.5 °C
    0 .. 15 °C           limit = dew point + 10 °C
    ambient < 0 °C       limit = 5 °C (fixed)

If the lens probe is above the limit, the firmware sets PWM1 to 0 and REFUSES PWM1 writes until
the probe drops back under the limit, then restores the previous level by itself. There is no
hysteresis, so a strong heater cycles on and off. Hold mode recognises this (from the refusal
ack and its own copy of the calculation) and reports it instead of treating it as a lost
setting; it keeps retrying quietly each poll, which is harmless.

Power readings
--------------
Power is the vendor app's formula (corrected INA219 current x input voltage + 0.269 W). The
firmware samples sensors every 5 s, and each INA219 reading is a sub-millisecond snapshot while
the dew heaters switch at 10 Hz, so a reading catches each heater fully on or fully off. "Max"
approximates every heater on at once; "Avg" (and Wh) is the better guide to consumption.

Protocol (firmware V1.3, disassembly-verified; full reference in docs/PROTOCOL.md)
----------------------------------------------------------------------------------
  Frame:  24 06 cmd idx val chk        chk = sum(bytes) % 255   (0..254, never 0xFF)
  Reply:  24 len cmd payload... chk    len = payload + 4 (6, 8 or 14)
  01 idx val   set: idx 0-6 outputs (val 00/FF), 7 ADJ, 8 PWM1, 9 PWM2 (val raw);
               ack 24 06 01 idx FF|00 chk (00 = refused)
  03 / 04 / 05 / 06 / 07   input V / lens probe / ambient / RH / current
  08           state: 7 output bytes, ADJ, PWM1, PWM2
  FE FE / FF FF            all outputs on / off
  Exchange is half-duplex: flush input, send one 6-byte frame, read one fixed-length reply.
  Bad frames (wrong checksum) are dropped silently; there is no NAK, only a missing reply.
  Sensor payload b3..b6 is base-255, not base-256: x = ((b3*255 + b4)*255 + b5)*255 + b6,
  value = x / 100; temperatures and RH carry a +255 offset. (The old big-endian decode read
  about 0.6 °C and 2.2 %RH high.) The firmware's upper two bytes are not true base-255 digits,
  but they are 0 for every value this script reads (x < 65025), so the decode is exact here.
  DTR/RTS must be low before the port opens, or the ESP32 resets. The hub sends nothing
  unprompted except its "SV241Pro ..." boot banner, so seeing that means it rebooted.

Hardware status (2026-11-21, COM6): reads, PWM2 write/read-back, hold mode, base-255 decode,
04 = lens / 05 = ambient and the PWM1 cutoff formula are confirmed. DC/USB output switching
(on/off, all on/off) follows the firmware disassembly but is NOT yet tested on hardware.

Known unknowns: the exact ADJ OUT volts-per-raw (vendor app's 15.3 V at raw 253 is used; the
firmware only sets a 40 kHz PWM duty, so the real scale depends on the board's regulator), and
the scale of cmd 02 (INA219 power, unused here). Use raw:N when the ADJ voltage matters.

Code layout (sections in file order)
------------------------------------
  Protocol helpers     frame(), checksum_ok(), base255(), dew_point(), cutoff_limit()
  Port discovery       list_serial_ports(), pick_port(), PosixSerial (stdlib termios backend)
  Serial link          Hub: open/verify/close, xfer(), state()/sensors() reads, verified writes
  Parsing/formatting   parse_level() (V / % / raw:N -> raw), fmt_level(), parse_outputs()
  One-shot commands    cmd_status, cmd_sensors, cmd_switch, cmd_set, cmd_raw, cmd_ports
  Hold mode            Stat, Holder (poll, enforce, cutoff tracking, CSV, render), cmd_hold
  TUI mode             Keys (raw key reader), Tui (keyboard actions on top of Holder), cmd_tui
  CLI                  build_parser(), insert_default_command(), main()
tests/fake_hub.py simulates the hub for tests/test_sv241.py (python -m unittest discover -s tests).

Requires Python 3.8+. Windows needs pyserial (pip install pyserial); on macOS and Linux it is
optional: without it a built-in termios backend is used and ports are found via /dev + sysfs.
Not affiliated with SVBONY.
"""
import argparse
import collections
import csv
import errno
import json
import math
import os
import re
import shutil
import signal
import sys
import time

# --- Hardware / protocol constants ---------------------------------------------------------
BAUD = 115200                     # CH340 line rate; 8N1, no flow control
CH340 = (0x1A86, 0x7523)          # USB VID:PID of the hub's WCH CH340 bridge (auto-detect key)
# Output names in hub index order: CMD 01 idx 0..6 and bytes 3..9 of the CMD 08 state reply.
# idx 0 = DC1 follows the firmware GPIO table; not yet confirmed by a physical bench check.
OUTPUTS = ["DC1", "DC2", "DC3", "DC4", "DC5", "USB-C", "USB-A"]
OUT_ALIASES = {n.lower().replace("-", ""): i for i, n in enumerate(OUTPUTS)}   # dc1, usbc, ...
OUT_ALIASES.update({str(i + 1): i for i in range(5)})          # "1".."5" -> DC1..DC5
CHANNELS = {"adj": 7, "pwm1": 8, "pwm2": 9}                    # hub index for CMD 01
# Levels are raw 0..255 (firmware LEDC duty = round(raw/255 * 1023)). The vendor app calls 253
# "100 %" and "15.3 V"; those are app conventions, kept so values match what users see there.
RAW_FULL = 253           # vendor app's 100 % / 15.3 V point
RAW_MAX = 255            # firmware accepts the full byte range
ADJ_VMAX = 15.3          # assumed ADJ OUT volts at RAW_FULL (linear scale; unverified on hardware)
INA_SCALE = 15.57        # vendor app's INA219 current correction (board shunt != library default)
PWR_OFFSET_MW = 269.39   # vendor app's fixed power offset (mW)
COMMANDS = ("status", "sensors", "on", "off", "set", "raw", "ports", "hold", "tui")


class CommError(Exception):
    """Serial-level failure: port missing/busy, I/O error, or no valid reply. Exit code 1."""


class Refused(Exception):
    """The hub answered but did not take a write (read-back mismatch or refusal ack). Exit code 2."""


# ------------------------------------------------------------------------------------------
# Protocol helpers
# ------------------------------------------------------------------------------------------
def frame(cmd, idx=0, val=0):
    """Build a 6-byte request: 24 06 cmd idx val chk.

    chk = sum of the first five bytes mod 255 (0..254). The firmware compares against the true
    remainder, so a frame summing to exactly 255 must carry 0x00; the vendor app's 0xFF there is
    silently dropped (svbony-probe.py demonstrates this). Read commands ignore idx/val, and the
    all-on/off commands need idx to repeat the command byte (frame(0xFE, 0xFE))."""
    b = [0x24, 0x06, cmd & 0xFF, idx & 0xFF, val & 0xFF]
    return bytes(b + [sum(b) % 255])


def checksum_ok(r):
    """True if reply r (any length >= 4) ends in sum(r[:-1]) % 255. Does not check 24/len/cmd."""
    return len(r) >= 4 and sum(r[:-1]) % 255 == r[-1]


def base255(bs):
    """Decode sensor payload bytes as base-255 digits, most significant first.

    The firmware builds each byte as a quotient mod 255 of x = trunc(value * 100). For x < 65025
    (every voltage, temperature, RH and current reading) the top two bytes are 0 and this is exact.
    It is not exact for large cmd 02 power values, which this script does not use (see
    docs/PROTOCOL.md, "Sensor value encoding")."""
    x = 0
    for b in bs:
        x = x * 255 + b
    return x


def dew_point(t, rh):
    """Dew point (°C) from temperature t (°C) and relative humidity rh (%), or None.

    Magnus with a = 17.27, b = 237.7, taken from earlier client code (docs/PROTOCOL.md tags them
    [C], not read out of the firmware); the cutoff predictions made with them matched the hub.
    RH is clamped to 100; RH <= 0 returns None (the firmware then uses -100 °C, which can never
    trip the cutoff)."""
    if t is None or rh is None or rh <= 0:
        return None
    a, b = 17.27, 237.7
    g = math.log(min(rh, 100.0) / 100.0) + a * t / (b + t)
    return b * g / (a - g)


def cutoff_limit(ambient, rh):
    """Lens temperature above which the firmware forces PWM1 to 0 (None if unknown).

    Mirrors the firmware routine at 0x400d19ac (run every 5 s): dew point + 7.5 °C above 15 °C
    ambient, + 10 °C from 0 to 15 °C, fixed 5 °C below 0 °C. Ambient is the SHT4x reading, which
    already has the firmware's -4 °C correction applied; do not subtract it again. Confirmed on
    hardware: the cutoff fired exactly when the lens crossed this computed limit."""
    if ambient is None:
        return None
    if ambient < 0:
        return 5.0
    dp = dew_point(ambient, rh)
    return None if dp is None else dp + (7.5 if ambient > 15 else 10.0)


# ------------------------------------------------------------------------------------------
# Port discovery and a stdlib serial backend for macOS / Linux
# ------------------------------------------------------------------------------------------
def have_pyserial():
    """True if pyserial imports. Required on Windows; optional on macOS/Linux (PosixSerial)."""
    try:
        import serial  # noqa: F401
        return True
    except ImportError:
        return False


def _linux_usb_ids(dev):
    """(vid, pid, description) of a Linux tty from sysfs, or (None, None, '').

    /sys/class/tty/<name>/device points at the USB *interface*; idVendor/idProduct live on the
    parent USB device node, so walk up a few levels until they appear."""
    import glob
    base = f"/sys/class/tty/{os.path.basename(dev)}/device"
    try:
        p = os.path.realpath(base)
    except OSError:
        return None, None, ""
    for _ in range(6):                      # walk up from the interface to the USB device node
        try:
            with open(os.path.join(p, "idVendor")) as f:
                vid = int(f.read(), 16)
            with open(os.path.join(p, "idProduct")) as f:
                pid = int(f.read(), 16)
            prod = ""
            for name in ("product", "manufacturer"):
                for fn in glob.glob(os.path.join(p, name)):
                    with open(fn) as f:
                        prod = prod or f.read().strip()
            return vid, pid, prod
        except (OSError, ValueError):
            p = os.path.dirname(p)
    return None, None, ""


def posix_ports():
    """[(device, vid, pid, description)] for USB serial ports, without pyserial.
    Linux: /dev/ttyUSB*, /dev/ttyACM* with VID:PID from sysfs. macOS: /dev/cu.* USB serial nodes
    (Apple's CH34x driver: cu.usbserial-*, WCH's driver: cu.wchusbserial*); VID:PID is not
    available without pyserial/ioreg, so names that only the WCH/CH34x drivers use count as CH340."""
    import glob
    out = []
    if sys.platform.startswith("linux"):
        for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
            vid, pid, desc = _linux_usb_ids(dev)
            out.append((dev, vid, pid, desc or "USB serial"))
    elif sys.platform == "darwin":
        for dev in sorted(glob.glob("/dev/cu.*")):
            n = os.path.basename(dev)[3:].lower()
            if "wchusbserial" in n:
                out.append((dev, *CH340, "WCH CH34x driver"))
            elif n.startswith(("usbserial", "usbmodem")):
                out.append((dev, None, None, "USB serial"))
    return out


def list_serial_ports():
    """[(device, vid, pid, description)], via pyserial when installed, else posix_ports()."""
    if have_pyserial():
        from serial.tools import list_ports
        return [(p.device, p.vid, p.pid, p.description) for p in list_ports.comports()]
    if os.name == "nt":
        raise CommError("pyserial is required on Windows: pip install pyserial")
    return posix_ports()


def pick_port(ports):
    """Choose the hub's port from list_serial_ports() output. Returns (device, note or None).

    Order: CH340 VID:PID matches, preferring macOS cu.* over tty.* (tty.* open blocks waiting
    for carrier detect). With no CH340 match, a single USB-serial-looking device is returned as
    a guess, flagged by a note containing "trying"; Hub._verify_guess() then requires it to
    answer a read-only state query before anything is written. Non-USB ports (COM1, ttyS*,
    Bluetooth) are never guessed. Raises CommError if nothing fits."""
    def pref(dev):
        return (0 if "/cu." in dev else 1, dev)
    hits = sorted((d for d, v, p, _ in ports if (v, p) == CH340), key=pref)
    if sys.platform == "darwin":
        hits = [d for d in hits if "/tty." not in d] or hits
    if hits:
        return hits[0], (f"several CH340 ports {hits}, using {hits[0]}" if len(hits) > 1 else None)
    usb = [d for d, v, p, desc in ports if v is None and "/tty." not in d
           and os.path.basename(d).startswith(("ttyUSB", "ttyACM", "cu.usbserial", "cu.usbmodem"))]
    if len(usb) == 1:
        return usb[0], f"no port identified as CH340; trying the only USB serial port {usb[0]}"
    raise CommError("no CH340 (1A86:7523) port found; pass --port"
                    + (f" (USB serial ports seen: {', '.join(usb)})" if usb else ""))


class PosixSerial:
    """Minimal termios serial port with the pyserial surface Hub uses: read(n) with a timeout,
    write, flush, reset_input_buffer, in_waiting, close. 8N1, no flow control, raw mode.
    DTR/RTS: the kernel raises both on open (pyserial on POSIX does the same before it can lower
    them); they are dropped immediately after, and HUPCL is cleared so close() doesn't pulse them.
    Raising both together does not trigger the ESP32 auto-reset circuit, which needs one without
    the other. (That relies on the standard ESP32 two-transistor auto-reset wiring; it is not yet
    confirmed on this hub, so treat a "hub REBOOTED" event on POSIX open as a sign it is wrong.)"""

    def __init__(self, port, baudrate=BAUD, timeout=0.05):
        """Open and configure `port`; raises CommError (busy, permission, not a tty).

        O_NONBLOCK keeps open() from waiting on carrier detect; reads still honour `timeout` via
        select() in read(). O_NOCTTY stops the port becoming our controlling terminal."""
        import fcntl
        import termios
        self.port, self.timeout = port, timeout
        try:
            self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            if e.errno == errno.EBUSY:
                raise CommError(f"{port} is in use by another program")
            if not isinstance(e, PermissionError):
                raise CommError(f"can't open {port}: {e}")
            hint = (" (add yourself to the 'dialout' group, or 'uucp' on Arch, then log in again)"
                    if sys.platform.startswith("linux") else "")
            raise CommError(f"can't open {port}: {e}{hint}")
        try:
            try:
                fcntl.ioctl(self.fd, termios.TIOCEXCL)     # refuse other openers (non-root)
            except OSError:
                pass
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise CommError(f"{port} is in use by another program")
            except OSError:
                pass                                       # device doesn't support flock
            # (flock only stops other flock users, e.g. a second copy of this script; TIOCEXCL
            # above is what makes other programs' opens fail with EBUSY.)
            self._lines(False)                             # drop DTR/RTS as early as possible
            attr = termios.tcgetattr(self.fd)
            # Raw mode: no input translation, no software flow control, no output processing,
            # no echo/canonical line editing/signals. CLOCAL ignores modem status lines; HUPCL
            # cleared so close() does not toggle DTR (which could reset the ESP32).
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attr
            iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP | termios.INLCR
                       | termios.IGNCR | termios.ICRNL | termios.IXON | termios.IXOFF | termios.IXANY)
            oflag &= ~termios.OPOST
            lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
            cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL
                       | getattr(termios, "CRTSCTS", 0))
            cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
            speed = getattr(termios, f"B{baudrate}")
            cc = list(cc)
            cc[termios.VMIN], cc[termios.VTIME] = 0, 0     # reads return at once; select() times them
            termios.tcsetattr(self.fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
            self._lines(False)                             # again, in case tcsetattr re-raised them
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except termios.error as e:
            os.close(self.fd)
            raise CommError(f"can't configure {port}: {e} (not a serial port?)")
        except Exception:
            os.close(self.fd)
            raise

    def _lines(self, on):
        """Raise (on=True) or drop DTR and RTS together via TIOCMBIS/TIOCMBIC; best effort."""
        import fcntl
        import struct
        import termios
        bits = getattr(termios, "TIOCM_DTR", 0x002) | getattr(termios, "TIOCM_RTS", 0x004)
        req = getattr(termios, "TIOCMBIS" if on else "TIOCMBIC", None)
        if req is None:
            return
        try:
            fcntl.ioctl(self.fd, req, struct.pack("I", bits))
        except OSError:
            pass                                          # ptys and some adapters lack modem lines

    @property
    def in_waiting(self):
        """Bytes buffered for reading (FIONREAD); 0 if the driver can't say."""
        import fcntl
        import struct
        import termios
        try:
            return struct.unpack("I", fcntl.ioctl(self.fd, termios.FIONREAD, b"\0\0\0\0"))[0]
        except OSError:
            return 0

    def read(self, n):
        """Read up to n bytes, returning early with what arrived when self.timeout expires.
        An empty os.read() after select() says readable means the device went away (USB
        unplugged); that raises OSError, which Hub.xfer turns into CommError and a re-open."""
        import select
        buf = bytearray()
        end = time.monotonic() + self.timeout
        while len(buf) < n:
            left = end - time.monotonic()
            if left <= 0:
                break
            r, _, _ = select.select([self.fd], [], [], left)
            if not r:
                break
            try:
                chunk = os.read(self.fd, n - len(buf))
            except BlockingIOError:
                continue
            if not chunk:
                raise OSError(f"{self.port} disconnected")
            buf += chunk
        return bytes(buf)

    def write(self, data):
        """Write all of data (the fd is non-blocking, so loop on partial writes); 2 s overall limit."""
        import select
        data = memoryview(bytes(data))
        total, end = len(data), time.monotonic() + 2.0
        while data:
            if time.monotonic() > end:
                raise OSError(f"write to {self.port} timed out")
            select.select([], [self.fd], [], 0.5)
            try:
                k = os.write(self.fd, data)
            except BlockingIOError:
                continue
            data = data[k:]
        return total

    def flush(self):
        """Wait until queued output has been transmitted (tcdrain), like pyserial's flush()."""
        import termios
        try:
            termios.tcdrain(self.fd)
        except OSError:
            pass

    def reset_input_buffer(self):
        """Discard unread input (stale replies) before a new request."""
        import termios
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            finally:
                self.fd = None


# ------------------------------------------------------------------------------------------
# Serial link
# ------------------------------------------------------------------------------------------
class Hub:
    """Serial link to one hub. Opens lazily, re-opens after USB drops, spots the boot banner
    after a reset.

    Every exchange goes through xfer(): one request frame, one fixed-length reply. Any I/O error
    closes the port and raises CommError; the next call re-opens it, so long-running modes
    (hold, tui) survive USB drops. Writes (set_output/set_all/set_channel) are verified by a
    CMD 08 read-back and raise Refused when the hub did not take the value.

    `reboot_seen` latches when the "SV241Pro" boot banner shows up in unsolicited input; the
    hold loop reports and clears it. Tests substitute this class via main(hub_factory=...)."""

    def __init__(self, port=None, timeout=0.4):
        """port: device name, or None to auto-detect on first open. timeout: reply wait (s)."""
        self.port_arg, self.timeout = port, timeout
        self.port = None
        self.ser = None
        self.reboot_seen = False

    @staticmethod
    def find_port():
        """Auto-detect via pick_port(); prints any note to stderr.
        Sets the class attribute Hub._guessed when the port was picked by name only, so the next
        open() verifies it (class-level because this is a staticmethod; cleared once verified)."""
        dev, note = pick_port(list_serial_ports())
        Hub._guessed = note is not None and "trying" in note
        if note:
            print(f"note: {note}", file=sys.stderr)
        return dev

    def open(self):
        """Open the port if not already open; returns self. Uses pyserial when installed, else
        PosixSerial (not on Windows). The serial-level timeout is short (0.05 s); xfer() applies
        the overall reply timeout. After opening: 0.3 s settle, then check for a boot banner."""
        if self.ser:
            return self
        self.port = self.port_arg or self.find_port()
        if not have_pyserial():
            if os.name == "nt":
                raise CommError("pyserial is required on Windows: pip install pyserial")
            self.ser = PosixSerial(self.port, BAUD, 0.05)   # stdlib termios backend
            time.sleep(0.3)
            self._scan_unsolicited()
            return self._verify_guess()
        import serial
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, BAUD, 0.05
        s.bytesize, s.parity, s.stopbits = 8, "N", 1
        s.xonxoff = s.rtscts = s.dsrdtr = False
        # Must be set before open(): pyserial applies them as part of opening. Asserting DTR/RTS
        # drives the ESP32 auto-reset circuit and reboots the hub (docs/PROTOCOL.md).
        s.dtr = s.rts = False
        try:
            s.open()
        except serial.SerialException as e:
            raise CommError(f"can't open {self.port}: {e} (another program holding it?)")
        self.ser = s
        time.sleep(0.3)
        self._scan_unsolicited()
        return self._verify_guess()

    def _verify_guess(self):
        """A port picked by name only (no VID:PID match) must answer a read-only state query
        before anything is written to it."""
        if not getattr(Hub, "_guessed", False) or self.port_arg:
            return self
        try:
            self.state(tries=2)
        except CommError:
            self.close()
            raise CommError(f"{self.port} (guessed) did not answer like an SV241 Pro; pass --port")
        Hub._guessed = False
        return self

    def close(self):
        """Close the port if open; never raises. Safe to call repeatedly."""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def _scan_unsolicited(self):
        """Consume any bytes nobody asked for. The hub only sends unprompted output at boot
        ("SV241Pro V1.3 ..."), so finding that text means it reset; latch reboot_seen."""
        n = getattr(self.ser, "in_waiting", 0)
        if n:
            if b"SV241Pro" in self.ser.read(n):
                self.reboot_seen = True

    def xfer(self, tx, expect):
        """Send frame tx, return up to `expect` reply bytes (fewer on timeout; caller validates).

        Half-duplex: input is flushed first so a late reply to an earlier request can't be taken
        for this one. Any exception closes the port and becomes CommError."""
        self.open()
        try:
            self._scan_unsolicited()   # look for a boot banner before discarding the buffer
            self.ser.reset_input_buffer()
            self.ser.write(tx)
            self.ser.flush()
            buf = bytearray()
            end = time.monotonic() + self.timeout
            while len(buf) < expect and time.monotonic() < end:
                buf += self.ser.read(expect - len(buf))
        except Exception as e:
            self.close()
            raise CommError(f"serial I/O failed on {self.port}: {e}")
        return bytes(buf)

    # --- reads ---------------------------------------------------------------------------
    def state(self, tries=3):
        """CMD 08. Returns (outputs, levels): outputs = 7 bools in OUTPUTS order, levels =
        {"adj", "pwm1", "pwm2": raw 0..255}. Reply is 14 bytes: 24 0E 08 o0..o6 adj pwm1 pwm2 chk
        (outputs reported 0/1 here, although CMD 01 sets them with 00/FF). Retries, then CommError."""
        r = b""
        for _ in range(tries):
            r = self.xfer(frame(0x08), 14)
            if len(r) == 14 and r[0] == 0x24 and r[2] == 0x08 and checksum_ok(r):
                return [v != 0 for v in r[3:10]], {"adj": r[10], "pwm1": r[11], "pwm2": r[12]}
        raise CommError(f"bad state reply: {r.hex(' ') or 'nothing'}")

    def sensor(self, cmd, tries=2):
        """Read one sensor command (03..07). Reply is 8 bytes: 24 08 cmd b3 b4 b5 b6 chk.
        Returns the decoded value / 100, still including the +255 offset for 04/05/06, or None
        after `tries` bad replies (a missing reading is not a comm error)."""
        for _ in range(tries):
            r = self.xfer(frame(cmd), 8)
            if len(r) == 8 and r[0] == 0x24 and r[2] == cmd and checksum_ok(r):
                return base255(r[3:7]) / 100.0
        return None

    def sensors(self):
        """Read all sensors and derive the rest. Returns a dict (None where unavailable):
        input_v, current_a, power_w, lens_c, ambient_c, humidity_pct, dewpoint_c, pwm1_cutoff_c.
        Exception: with no INA219 fitted the firmware sends -1 for 03/07, which decodes as +1.56
        here; that case is not detected, so input_v ~ 1.56 V means "no power sensor", not None.

        04 = lens probe (DS18B20), 05 = ambient (SHT4x): both confirmed on hardware. Current and
        power use the vendor app's correction (INA_SCALE, PWR_OFFSET_MW). The firmware samples
        sensors every 5 s, so polling faster returns repeated values."""
        vin = self.sensor(0x03)
        ta = self.sensor(0x04)
        tb = self.sensor(0x05)
        rh = self.sensor(0x06)
        cur = self.sensor(0x07)
        lens = ta - 255 if ta is not None else None
        if lens is not None and not -50 <= lens <= 90:
            lens = None                        # unplugged probe reads about -127 °C
        amb = tb - 255 if tb is not None else None
        rh = rh - 255 if rh is not None else None
        cur_a = cur * INA_SCALE / 1000.0 if cur is not None else None
        pwr = ((INA_SCALE * vin * cur + PWR_OFFSET_MW) / 1000.0
               if vin is not None and cur is not None else None)
        dp = dew_point(amb, rh)
        return dict(input_v=vin, current_a=cur_a, power_w=pwr, lens_c=lens, ambient_c=amb,
                    humidity_pct=rh, dewpoint_c=dp, pwm1_cutoff_c=cutoff_limit(amb, rh))

    # --- writes --------------------------------------------------------------------------
    def set_raw(self, idx, val):
        """Send CMD 01 (idx 0-6 output with val 00/FF, 7-9 level with raw val); return the ack:
        'ok' (24 06 01 idx FF chk), 'refused' (24 06 01 idx 00 chk, status byte 00: e.g. PWM1
        during the dew cutoff, or an output value other than 00/FF) or 'noack' (missing or
        malformed reply). The ack alone is
        not trusted as proof; callers that care read the state back."""
        ack = self.xfer(frame(0x01, idx, val), 6)
        if (len(ack) == 6 and ack[0] == 0x24 and ack[1] == 0x06 and ack[2] == 0x01
                and ack[3] == idx and checksum_ok(ack)):
            return "ok" if ack[4] == 0xFF else "refused"
        return "noack"

    def set_output(self, idx, on):
        """Switch output idx (0..6) on/off, verified via CMD 08; raises Refused on mismatch.
        NOT yet tested on hardware (firmware-derived). Outputs take 00/FF, never 0/1."""
        ack = self.set_raw(idx, 0xFF if on else 0x00)
        time.sleep(0.05)
        outs, _ = self.state()
        if outs[idx] != on:
            raise Refused(f"{OUTPUTS[idx]} still {'on' if outs[idx] else 'off'} (ack {ack})")

    def set_all(self, on):
        """Firmware all-on (24 06 FE FE 00 28) / all-off (24 06 FF FF 00 2A); PWM/ADJ untouched.
        The firmware runs this as an RTOS task, hence the longer wait; the ack is ignored and the
        result checked by read-back. NOT yet tested on hardware."""
        self.xfer(frame(0xFE, 0xFE) if on else frame(0xFF, 0xFF), 6)
        time.sleep(0.1)
        outs, _ = self.state()
        bad = [OUTPUTS[i] for i, v in enumerate(outs) if v != on]
        if bad:
            raise Refused(f"still {'off' if on else 'on'}: {', '.join(bad)}")

    def set_channel(self, name, raw):
        """Set adj/pwm1/pwm2 to raw 0..255, verified by read-back (the firmware's duty round trip
        returns the same raw value). Raises Refused, naming the PWM1 dew cutoff when it is why."""
        ack = self.set_raw(CHANNELS[name], raw)
        time.sleep(0.05)
        _, dev = self.state()
        if dev[name] != raw:
            why = " (PWM1 dew cutoff: lens probe above limit)" if name == "pwm1" and ack == "refused" else ""
            raise Refused(f"{name.upper()} reads raw {dev[name]}, wanted {raw} (ack {ack}){why}")


# ------------------------------------------------------------------------------------------
# Parsing / formatting
# ------------------------------------------------------------------------------------------
def parse_level(name, s):
    """User level -> raw 0..255. "raw:N" (int, any base: raw:0x78) is taken as-is; otherwise a
    number is volts for adj (optional trailing "v") or percent for pwm1/pwm2 (optional "%"),
    scaled so 100 % / ADJ_VMAX = RAW_FULL (253). Raises ValueError for non-finite or
    out-of-range input. ADJ volts are approximate (see ADJ_VMAX)."""
    s = str(s).strip().lower()
    if s.startswith("raw:"):
        raw = int(s[4:], 0)
    else:
        x = float(s.rstrip("v" if name == "adj" else "%"))
        if not math.isfinite(x):
            raise ValueError(f"{name}: {s} is not a number")
        raw = round(x * RAW_FULL / (ADJ_VMAX if name == "adj" else 100.0))
    if not 0 <= raw <= RAW_MAX:
        raise ValueError(f"{name}: {s} is outside raw 0..{RAW_MAX}")
    return raw


def arg_level(name):
    """argparse `type=` wrapper around parse_level() for channel `name`."""
    def f(s):
        try:
            return parse_level(name, s)
        except ValueError as e:
            raise argparse.ArgumentTypeError(str(e))
    return f


def fmt_level(name, raw):
    """Raw level -> display text: "12.10 V~" (the ~ marks the approximate ADJ scale) or "40.3 %"."""
    if raw is None:
        return "—"
    if name == "adj":
        return f"{raw * ADJ_VMAX / RAW_FULL:.2f} V~"
    return f"{raw * 100.0 / RAW_FULL:.1f} %"


def parse_outputs(tokens):
    """Output names (space- or comma-separated; case, '-' and '_' ignored) -> sorted unique
    indexes 0..6. A lone "all" is handled by the caller (firmware all-on/off command) and never
    reaches here; "all" mixed with other names is rejected as unknown."""
    idx = []
    for t in tokens:
        for part in t.split(","):
            k = part.strip().lower().replace("-", "").replace("_", "")
            if not k:
                continue
            if k not in OUT_ALIASES:
                raise ValueError(f"unknown output '{part}' (use dc1..dc5, usbc, usba, all)")
            idx.append(OUT_ALIASES[k])
    return sorted(set(idx))


def show(v, unit, spec="{:.2f}"):
    """Format a reading with its unit, or "n/a" for None."""
    return f"{spec.format(v)} {unit}" if v is not None else "n/a"


def rounded(d):
    """Round every float in a sensors() dict to 3 places for JSON output (None stays None)."""
    return {k: (round(v, 3) if v is not None else None) for k, v in d.items()}


# ------------------------------------------------------------------------------------------
# One-shot commands
# ------------------------------------------------------------------------------------------
# Each cmd_* handler takes (hub, args) and returns the process exit code; main() maps
# CommError -> 1 and Refused -> 2 for anything they let escape.
def print_sensors(s):
    """Human-readable sensors() dict, including the PWM1 cutoff margin when computable."""
    print(f"  Input voltage  {show(s['input_v'], 'V')}")
    print(f"  Current        {show(s['current_a'], 'A', '{:.3f}')}")
    print(f"  Power          {show(s['power_w'], 'W')}   (instantaneous; heaters switch at 10 Hz)")
    print(f"  Lens probe     {show(s['lens_c'], '°C')}" + ("   (not connected)" if s["lens_c"] is None else ""))
    print(f"  Ambient        {show(s['ambient_c'], '°C')}")
    print(f"  Humidity       {show(s['humidity_pct'], '%')}")
    print(f"  Dew point      {show(s['dewpoint_c'], '°C')}")
    lim, lens = s["pwm1_cutoff_c"], s["lens_c"]
    if lim is not None and lens is not None:
        state = "ACTIVE: firmware holds PWM1 at 0" if lens > lim else f"{lim - lens:.2f} °C margin"
        print(f"  PWM1 cutoff    {lim:.2f} °C lens limit, {state}")


def cmd_status(hub, a):
    """status [--json] [--no-sensors]: outputs, ADJ/PWM levels and (by default) sensors."""
    outs, dev = hub.state()
    s = None if a.no_sensors else hub.sensors()
    if a.json:
        out = {"port": hub.port,
               "outputs": {n: v for n, v in zip(OUTPUTS, outs)},
               "channels": {k: {"raw": v, "value": fmt_level(k, v)} for k, v in dev.items()}}
        if s:
            out["sensors"] = rounded(s)
        print(json.dumps(out, indent=2, ensure_ascii=False))
        return 0
    print(f"SV241 Pro @ {hub.port}")
    print("Outputs   " + "  ".join(f"{n} {'ON' if v else 'off'}" for n, v in zip(OUTPUTS, outs)))
    for k, v in dev.items():
        print(f"  {k.upper():<5} {fmt_level(k, v):>9}   raw {v}")
    if s:
        print("Sensors")
        print_sensors(s)
    return 0


def cmd_sensors(hub, a):
    """sensors [--json]: sensor readings only."""
    s = hub.sensors()
    if a.json:
        print(json.dumps(rounded(s), indent=2))
    else:
        print_sensors(s)
    return 0


def cmd_switch(hub, a):
    """on/off <outputs...|all>. Each output is switched and verified separately; failures are
    reported and the rest still attempted (rc 2 if any failed). Untested on hardware."""
    on = a.cmd == "on"
    if [t.lower() for t in a.outputs] == ["all"]:
        hub.set_all(on)
        print(f"all outputs {'ON' if on else 'off'}")
        return 0
    rc = 0
    for i in parse_outputs(a.outputs):
        try:
            hub.set_output(i, on)
            print(f"{OUTPUTS[i]} {'ON' if on else 'off'}")
        except Refused as e:
            print(f"FAILED: {e}", file=sys.stderr)
            rc = 2
    return rc


def cmd_set(hub, a):
    """set adj|pwm1|pwm2 <level>: one verified write (Refused -> rc 2 via main())."""
    name = a.channel
    raw = parse_level(name, a.level)
    hub.set_channel(name, raw)
    print(f"{name.upper()} = {fmt_level(name, raw)} (raw {raw})")
    return 0


def cmd_raw(hub, a):
    """raw <cmd> [idx [val]] (hex): send one frame, print whatever arrives within the timeout.
    Debug aid: no validation of the command, and the reply is shown even if malformed."""
    vals = [int(x, 16) for x in a.bytes]
    while len(vals) < 3:
        vals.append(0)
    tx = frame(*vals[:3])
    r = hub.xfer(tx, 64)
    print(f"tx {tx.hex(' ')}")
    print(f"rx {r.hex(' ') or '(nothing)'}" + ("" if not r or checksum_ok(r) else "   [checksum mismatch]"))
    return 0


def cmd_ports(_a):
    """ports: list serial ports (CH340s flagged) and what auto-detect would pick. No hub needed."""
    ports = list_serial_ports()
    for dev, vid, pid, desc in ports:
        tag = "  <- CH340 (SV241 Pro?)" if (vid, pid) == CH340 else ""
        ids = f"{vid:04X}:{pid:04X}" if vid is not None else "????:????"
        print(f"{dev:<28} {ids}  {desc}{tag}")
    if not ports:
        print("no serial ports found")
    try:
        print(f"auto-detect picks: {pick_port(ports)[0]}")
    except CommError as e:
        print(f"auto-detect: {e}")
    return 0


# ------------------------------------------------------------------------------------------
# Hold mode
# ------------------------------------------------------------------------------------------
class Stat:
    """Running min / max (with the time each occurred) and mean of one reading."""

    def __init__(self):
        self.min = self.max = None
        self.t_min = self.t_max = None
        self.total = 0.0
        self.n = 0

    def add(self, v):
        if v is None:
            return
        now = time.strftime("%H:%M:%S")
        if self.max is None or v > self.max:
            self.max, self.t_max = v, now
        if self.min is None or v < self.min:
            self.min, self.t_min = v, now
        self.total += v
        self.n += 1

    @property
    def avg(self):
        return self.total / self.n if self.n else None


# (key in Hub.sensors(), label, unit, format), in display order
SENSOR_ROWS = [
    ("power_w", "Power", "W", "{:6.2f}"),
    ("current_a", "Current", "A", "{:6.3f}"),
    ("input_v", "Input voltage", "V", "{:6.2f}"),
    ("lens_c", "Lens probe", "°C", "{:6.2f}"),
    ("ambient_c", "Ambient", "°C", "{:6.2f}"),
    ("humidity_pct", "Humidity", "%", "{:6.2f}"),
    ("dewpoint_c", "Dew point", "°C", "{:6.2f}"),
]


class Holder:
    """Hold-mode engine: poll the hub, re-push targets it no longer reports, track the PWM1 dew
    cutoff, keep running stats/energy, optionally log CSV, and render the console display.
    The TUI reuses it unchanged for polling and for channels the user marks "held".

    targets: {channel: raw} to enforce (only channels given on the command line, or all three
    with --hold-current). status: per-channel text for the display. events: recent event lines.
    counts: re-pushes, failures, cutoffs, reboots and comm errors.

    PWM1 cutoff handling: while the firmware holds PWM1 at 0 it refuses writes, so a PWM1 that
    reads 0 with the refusal ack (or with our own cutoff_limit() predicting it) is reported as
    "FIRMWARE CUTOFF", not as a failed push. It is retried every poll (harmless: refused), and
    the firmware restores the saved level itself when the lens cools below the limit."""

    def __init__(self, a, hub):
        """a: parsed args (adj/pwm1/pwm2 targets, events, csv, no_clear, interval, hold_current)."""
        self.a = a
        self.hub = hub
        self.targets = {k: v for k, v in (("adj", a.adj), ("pwm1", a.pwm1), ("pwm2", a.pwm2))
                        if v is not None}
        self.status = {k: "" for k in CHANNELS}
        self.events = collections.deque(maxlen=a.events)
        self.counts = collections.Counter()
        self.in_cutoff = False            # PWM1 currently held at 0 by firmware
        self.started = time.time()
        self.stats = {k: Stat() for k, *_ in SENSOR_ROWS}
        self.energy_wh = 0.0              # integrated from power samples
        self._last_pwr = None             # (monotonic time, watts) of the previous sample
        self._last_event = ""
        self.csv = None
        if a.csv:
            new = not os.path.exists(a.csv)
            self.csv_f = open(a.csv, "a", newline="", encoding="utf-8")
            self.csv = csv.writer(self.csv_f)
            if new:
                self.csv.writerow(["time", "adj_raw", "pwm1_raw", "pwm2_raw", "vin", "current_a", "power_w",
                                   "lens_c", "ambient_c", "humidity", "dewpoint_c", "cutoff_limit_c",
                                   "cutoff_predicted", "event"])

    def event(self, msg):
        """Record a timestamped event; with --no-clear also print it (service-friendly log)."""
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        self.events.append(line)
        self._last_event = msg
        if self.a.no_clear:
            print(line, flush=True)

    def poll(self):
        """One cycle: read state + sensors, report reboots/cutoff transitions, enforce targets
        (write, wait 50 ms, read back), update stats/energy, write the CSV row. Returns the
        sensors dict plus outs/dev (dev reflects successful re-pushes). CommError propagates."""
        self._last_event = ""
        outs, dev = self.hub.state()
        s = self.hub.sensors()
        lens, limit = s["lens_c"], s["pwm1_cutoff_c"]
        predicted = lens is not None and limit is not None and lens > limit

        if self.hub.reboot_seen:
            self.hub.reboot_seen = False
            self.counts["reboot"] += 1
            self.event("hub REBOOTED (boot banner seen); settings were reloaded from its flash")

        # Cutoff entry/exit detection. Entry: PWM1 reads 0 while our formula predicts the cutoff.
        # Exit differs: unenforced PWM1 counts as released once it is non-zero again; an enforced
        # one once it is back at its target (the firmware restores the level saved at entry).
        if "pwm1" not in self.targets:
            # Monitor only: still report the firmware cutoff, from PWM1 reading 0 while predicted.
            if predicted and not self.in_cutoff and dev["pwm1"] == 0:
                self.in_cutoff = True
                self.counts["cutoff"] += 1
                self.event(f"PWM1 forced to 0 by firmware dew cutoff: lens {lens:.2f} °C > limit {limit:.2f} °C")
            elif self.in_cutoff and dev["pwm1"] > 0:
                self.in_cutoff = False
                self.event(f"firmware released PWM1 cutoff (lens {show(lens, '°C')}); "
                           f"back at {fmt_level('pwm1', dev['pwm1'])}")
        else:
            if predicted and not self.in_cutoff and dev["pwm1"] == 0:
                self.in_cutoff = True
                self.counts["cutoff"] += 1
                self.event(f"PWM1 forced to 0 by firmware dew cutoff: lens {lens:.2f} °C > limit {limit:.2f} °C")
            elif self.in_cutoff and dev["pwm1"] == self.targets["pwm1"]:
                self.in_cutoff = False
                self.event(f"firmware released PWM1 cutoff (lens {show(lens, '°C')}); "
                           f"back at {fmt_level('pwm1', dev['pwm1'])}")

        for name, want in self.targets.items():
            got = dev[name]
            if got == want:
                self.status[name] = "ok"
                continue
            res = self.hub.set_raw(CHANNELS[name], want)
            time.sleep(0.05)
            _, dev2 = self.hub.state()
            N = name.upper()
            if dev2[name] == want:
                self.counts[f"push_{name}"] += 1
                self.status[name] = "re-pushed"
                if name == "pwm1":
                    self.in_cutoff = False
                self.event(f"{N} was {fmt_level(name, got)} (raw {got}); re-pushed "
                           f"{fmt_level(name, want)} (raw {want}), verified")
                dev[name] = want
            # Push did not take. For PWM1 a refusal ack or a predicted cutoff means the firmware
            # is holding it at 0 on purpose: report once, keep retrying quietly each poll.
            elif name == "pwm1" and (res == "refused" or predicted):
                why = (f"lens {lens:.2f} °C > limit {limit:.2f} °C" if predicted
                       else "write refused by firmware")
                self.status[name] = f"FIRMWARE CUTOFF ({why})"
                if not self.in_cutoff:
                    self.in_cutoff = True
                    self.counts["cutoff"] += 1
                    self.event(f"PWM1 held at 0 by firmware dew cutoff ({why}); will restore when it clears")
            else:
                self.counts[f"fail_{name}"] += 1
                self.status[name] = f"PUSH FAILED (ack {res}, hub reports raw {dev2[name]})"
                self.event(f"{N}: push of raw {want} failed (ack {res}); hub reports raw {dev2[name]}")

        for name in CHANNELS:
            if name not in self.targets:
                self.status[name] = "not enforced" + (" (FIRMWARE CUTOFF)" if name == "pwm1" and self.in_cutoff else "")

        if self.csv:
            r = lambda v, n=2: round(v, n) if v is not None else ""
            self.csv.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), dev["adj"], dev["pwm1"], dev["pwm2"],
                               r(s["input_v"]), r(s["current_a"], 3), r(s["power_w"]), r(lens),
                               r(s["ambient_c"]), r(s["humidity_pct"]), r(s["dewpoint_c"]), r(limit),
                               int(predicted), self._last_event])
            self.csv_f.flush()

        for k, *_ in SENSOR_ROWS:
            self.stats[k].add(s[k])
        # Energy: trapezoid between consecutive power samples; skip gaps (comm errors) > 3 polls.
        now, pwr = time.monotonic(), s["power_w"]
        if pwr is not None:
            if self._last_pwr and now - self._last_pwr[0] <= 3 * self.a.interval:
                self.energy_wh += (pwr + self._last_pwr[1]) / 2 * (now - self._last_pwr[0]) / 3600.0
            self._last_pwr = (now, pwr)

        return dict(s, outs=outs, dev=dev)

    def render(self, s, err=None):
        """Redraw the full-screen hold display (clear + home, ANSI). s is poll()'s result or None
        after a comm error (then only the header, error, counts and events are shown)."""
        if self.a.no_clear:
            return
        up = int(time.time() - self.started)
        L = [f"SV241 Pro @ {self.hub.port or '?'}   {time.strftime('%Y-%m-%d %H:%M:%S')}   "
             f"poll {self.a.interval:g} s   up {up // 3600:02d}:{up % 3600 // 60:02d}:{up % 60:02d}"]
        if err:
            L.append(f"\n!! {err} — retrying")
        if s:
            L.append("\nOutputs   " + "  ".join(f"{n} {'ON ' if v else 'off'}" for n, v in zip(OUTPUTS, s["outs"])))
            L.append(f"\n{'Channel':<9}{'Target':>12}{'Device':>12}   Status")
            for name in CHANNELS:
                L.append(f"{name.upper():<9}{fmt_level(name, self.targets.get(name)):>12}"
                         f"{fmt_level(name, s['dev'][name]):>12}   {self.status[name]}")
            L.append(f"\n{'Sensor':<15}{'Now':>10}{'Min':>11}{'at':>9}{'Max':>11}{'at':>9}{'Avg':>11}")
            for key, label, unit, spec in SENSOR_ROWS:
                st = self.stats[key]
                cell = lambda v: (spec.format(v) + " " + unit) if v is not None else "n/a"
                L.append(f"  {label:<13}{cell(s[key]):>10}{cell(st.min):>11}{st.t_min or '':>9}"
                         f"{cell(st.max):>11}{st.t_max or '':>9}{cell(st.avg):>11}")
            L.append(f"  Energy since start {self.energy_wh:8.2f} Wh")
            lim, lens = s["pwm1_cutoff_c"], s["lens_c"]
            if lim is not None and lens is not None:
                margin = lim - lens
                L.append(f"  PWM1 cutoff at {lim:7.2f} °C   lens is {abs(margin):.2f} °C "
                         f"{'BELOW' if margin >= 0 else 'ABOVE'} it"
                         + ("   -> firmware holds PWM1 at 0" if margin < 0 else ""))
            elif lens is None:
                L.append("  Lens probe not connected: firmware cutoff cannot trigger")
        c = self.counts
        L.append(f"\nCounts  re-pushes ADJ {c['push_adj']}  PWM1 {c['push_pwm1']}  PWM2 {c['push_pwm2']}   "
                 f"cutoffs {c['cutoff']}   push failures {c['fail_adj'] + c['fail_pwm1'] + c['fail_pwm2']}   "
                 f"hub reboots {c['reboot']}   comm errors {c['comm']}")
        L.append("\nRecent events" + ("" if self.events else ": none"))
        L.extend("  " + e for e in self.events)
        L.append("\nCtrl+C to quit (settings are left as they are)")
        sys.stdout.write("\033[2J\033[H" + "\n".join(L) + "\n")
        sys.stdout.flush()

    def run(self, max_polls=None):
        """Main hold loop: optionally adopt current levels as targets, then poll every
        --interval seconds until Ctrl+C (or max_polls, for tests). Closes the CSV on exit."""
        if self.a.hold_current:
            _, dev = self.hub.state()
            for name in CHANNELS:
                self.targets.setdefault(name, dev[name])
            if self.targets.get("pwm1") == 0:
                print("warning: PWM1 currently reads 0; if the firmware cutoff is active right now, "
                      "0 is being adopted as the target. Pass --pwm1 explicitly to avoid that.")
        desc = ", ".join(f"{k.upper()}={fmt_level(k, v)} (raw {v})" for k, v in self.targets.items()) \
            or "none (monitor only)"
        self.event(f"started; targets: {desc}")
        try:
            self._loop(max_polls)
        finally:
            if self.csv:
                self.csv_f.close()

    def _loop(self, max_polls):
        """Poll/render at a fixed cadence. A CommError is counted, shown and the port closed;
        the next poll re-opens it (Hub.xfer -> open), so USB drops are ridden out."""
        n = 0
        while max_polls is None or n < max_polls:
            n += 1
            t0 = time.monotonic()
            try:
                self.render(self.poll())
            except CommError as e:
                self.counts["comm"] += 1
                self.event(f"comm error: {e}")
                self.hub.close()
                self.render(None, str(e))
            if max_polls is None or n < max_polls:
                time.sleep(max(0.0, self.a.interval - (time.monotonic() - t0)))


def cmd_hold(hub, a):
    """hold (also the default command): run Holder until Ctrl+C, which leaves settings as-is."""
    if a.interval < 1:
        print("error: --interval must be at least 1 s", file=sys.stderr)
        return 2
    if os.name == "nt":
        os.system("")   # enables ANSI escape handling in the Windows console
    try:
        Holder(a, hub).run(max_polls=getattr(a, "max_polls", None))
    except KeyboardInterrupt:
        pass
    return 0


# ------------------------------------------------------------------------------------------
# TUI mode (interactive; stdlib only: ANSI escapes + msvcrt on Windows, termios/select on POSIX)
# ------------------------------------------------------------------------------------------
class Keys:
    """Non-blocking key reader. read(timeout) returns tokens: 'up', 'down', 'left', 'right',
    'enter', 'esc', 'backspace', 'tab', 'ctrl-c', or single printable characters."""

    WIN_SPECIAL = {"H": "up", "P": "down", "K": "left", "M": "right"}
    ANSI = {"A": "up", "B": "down", "C": "right", "D": "left"}

    def __enter__(self):
        """POSIX: put stdin in cbreak mode (per-key input, no echo) and remember the old mode.
        Windows needs no setup: msvcrt reads the console per key already."""
        self.saved = None
        if os.name != "nt":
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)          # keeps ISIG, so Ctrl+C still raises KeyboardInterrupt
        return self

    def __exit__(self, *exc):
        """Restore the saved terminal mode (POSIX)."""
        if self.saved is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    @staticmethod
    def _single(c):
        """Map one control character to its token name; printable characters pass through."""
        return {"\r": "enter", "\n": "enter", "\x1b": "esc", "\x7f": "backspace", "\x08": "backspace",
                "\x03": "ctrl-c", "\t": "tab"}.get(c, c)

    def read(self, timeout):
        """Wait up to `timeout` s for input; return all key tokens available (possibly [])."""
        return self._read_win(timeout) if os.name == "nt" else self._read_posix(timeout)

    def _read_win(self, timeout):
        import msvcrt
        end = time.monotonic() + timeout
        out = []
        while True:
            while msvcrt.kbhit():
                c = msvcrt.getwch()
                if c in ("\x00", "\xe0"):       # arrow / function key prefix
                    k = self.WIN_SPECIAL.get(msvcrt.getwch())
                    if k:
                        out.append(k)
                else:
                    out.append(self._single(c))
            if out or time.monotonic() >= end:
                return out
            time.sleep(0.02)

    def _read_posix(self, timeout):
        """Arrows arrive as ESC [ A..D (or ESC O A..D in application-cursor mode); anything else
        after ESC is reported as a bare 'esc' followed by the characters themselves."""
        import select
        r, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
        if not r:
            return []
        data = os.read(self.fd, 64).decode("utf-8", "ignore")
        if data.endswith(("\x1b", "\x1b[", "\x1bO")):      # arrow sequence split across reads
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r:
                data += os.read(self.fd, 64).decode("utf-8", "ignore")
        out, i = [], 0
        while i < len(data):
            c = data[i]
            if c == "\x1b" and i + 2 < len(data) and data[i + 1] in "[O" and data[i + 2] in self.ANSI:
                out.append(self.ANSI[data[i + 2]])
                i += 3
                continue
            out.append(self._single(c))
            i += 1
        return out


# Selectable rows: the 7 outputs, then the three level channels
TUI_ROWS = [("out", i) for i in range(len(OUTPUTS))] + [("ch", n) for n in CHANNELS]
# Left/Right step sizes in raw units: 0.5 V for ADJ (~8 raw), 5 % for PWM (~13 raw)
COARSE = {"adj": round(0.5 * RAW_FULL / ADJ_VMAX), "pwm1": round(0.05 * RAW_FULL), "pwm2": round(0.05 * RAW_FULL)}
PWM1_SOFT_MAX = round(0.80 * RAW_FULL)      # above this PWM1 tends to cycle at the cutoff edge
DEBOUNCE_S = 0.6                            # level keys are batched into one write after this idle time
REV, BOLD, DIM, YEL, RED, RST = "\033[7m", "\033[1m", "\033[2m", "\033[33m", "\033[31m", "\033[0m"
KEY_HELP = ["Up/Dn j/k select   Space/Enter toggle output   a all on   x all off   r refresh   q quit",
            "Left/Right -/+ step 0.5 V / 5 %   , . step 1 raw   e or Enter type a value   h hold/release"]


class Tui:
    """Live status display with keyboard control. Reuses Holder for polling, stats, CSV and
    (optional) enforcement: a channel marked 'hold' is re-pushed exactly as in hold mode.

    Level keys don't write immediately: steps accumulate in `pending` and are sent as one write
    DEBOUNCE_S after the last key (flush_pending), so holding an arrow key doesn't flood the hub.
    Typed values (e) and output switches are sent at once. Switching an output off, and all off,
    ask y/N first. Every write is followed by an immediate poll. keys/out are injectable for tests."""

    def __init__(self, a, hub, keys=None, out=None):
        """a: parsed tui args; hub: Hub (or test double); keys: Keys-like; out: text stream."""
        self.a, self.hub = a, hub
        self.holder = Holder(a, hub)
        self.keys = keys or Keys()
        self.out = out or sys.stdout
        self.sel = 0
        self.last = None            # last successful poll result
        self.err = None             # last comm error text, cleared by a good poll
        self.msg = ""               # one-line feedback under the tables
        self.prompt = None          # ("edit", name, buffer) | ("confirm", label, fn)
        self.pending = {}           # channel -> [raw, monotonic time of last key]
        self.quit = False
        self.next_poll = 0.0

    # --- helpers -------------------------------------------------------------------------
    def row(self):
        """(kind, which) of the selected row: ("out", index) or ("ch", channel name)."""
        return TUI_ROWS[self.sel]

    def note(self, text, log=True):
        """Show text on the message line; also add it to the event log unless log=False."""
        self.msg = text
        if log:
            self.holder.event(text)

    def current(self, name):
        """Channel's effective raw level: the pending (unsent) value if any, else last read."""
        if name in self.pending:
            return self.pending[name][0]
        return self.last["dev"][name] if self.last else None

    def act(self, fn, done):
        """Run one hub write; report the result; poll right after."""
        try:
            fn()
            self.note(done)
        except Refused as e:
            self.note(f"REFUSED: {e}")
        except CommError as e:
            self.holder.counts["comm"] += 1
            self.hub.close()
            self.note(f"comm error: {e}")
        self.next_poll = 0.0

    # --- actions -------------------------------------------------------------------------
    def set_output(self, i, on):
        """Switch one output (verified by Hub.set_output). Untested on hardware."""
        self.act(lambda: self.hub.set_output(i, on), f"{OUTPUTS[i]} switched {'ON' if on else 'off'}")

    def set_all(self, on):
        """Firmware all-on / all-off. Untested on hardware."""
        self.act(lambda: self.hub.set_all(on), f"all outputs switched {'ON' if on else 'off'}")

    def apply_level(self, name, raw):
        """Write a level now. If the channel is held, its hold target moves to the new value
        first, so the holder doesn't revert the user's change on the next poll."""
        t = self.holder.targets
        prev = t.get(name)
        if name in t:
            t[name] = raw                         # keep the holder from reverting the change

        def write():
            try:
                self.hub.set_channel(name, raw)
            except Refused:
                # A held PWM1 refused by the dew cutoff keeps its new target (the holder restores
                # it when the cutoff clears); any other refusal reverts to the previous target.
                if prev is not None and not (name == "pwm1" and self.holder.in_cutoff):
                    t[name] = prev
                raise
        self.act(write, f"{name.upper()} set to {fmt_level(name, raw)} (raw {raw})"
                 + (", held" if name in t else ""))

    def flush_pending(self, force=False):
        """Send pending levels idle for DEBOUNCE_S (all of them if force, e.g. on quit). A value
        equal to the last reading is dropped without a write."""
        now = time.monotonic()
        for name, (raw, t) in list(self.pending.items()):
            if force or now - t >= DEBOUNCE_S:
                del self.pending[name]
                if self.last is None or self.last["dev"][name] != raw:
                    self.apply_level(name, raw)

    def step(self, name, delta):
        """Queue current level + delta (debounced). PWM is capped at RAW_FULL (the app's 100 %);
        ADJ may go to RAW_MAX."""
        cur = self.current(name)
        if cur is None:
            self.note("no reading yet", log=False)
            return
        raw = max(0, min(RAW_MAX if name == "adj" else RAW_FULL, cur + delta))   # PWM: 100 % max
        self.pending[name] = [raw, time.monotonic()]
        self.msg = f"{name.upper()} -> {fmt_level(name, raw)} (raw {raw}), sending when you stop pressing"

    def toggle_hold(self, name):
        """Start holding a channel at its current level, or stop holding it. Holding PWM1 while
        the cutoff has it at 0 adopts 0 as the target, so that case is warned about."""
        t = self.holder.targets
        if name in t:
            del t[name]
            self.note(f"{name.upper()} no longer held")
            return
        cur = self.current(name)
        if cur is None:
            self.note("no reading yet", log=False)
            return
        t[name] = cur
        warn = ("  (PWM1 reads 0, possibly the firmware cutoff: 0 is now the target)"
                if name == "pwm1" and cur == 0 and self.holder.in_cutoff else "")
        self.note(f"{name.upper()} held at {fmt_level(name, cur)} (raw {cur}){warn}")

    def confirm(self, label, fn):
        """Ask y/N on the message line; fn runs only on y/Y (see _handle_prompt)."""
        self.prompt = ("confirm", label, fn)
        self.msg = ""

    # --- input ---------------------------------------------------------------------------
    def handle(self, k):
        """Dispatch one key token. An open prompt (confirm / value entry) takes all keys."""
        if self.prompt:
            return self._handle_prompt(k)
        kind, which = self.row()
        if k in ("q", "Q", "ctrl-c"):
            self.quit = True
        elif k in ("up", "k"):
            self.sel = (self.sel - 1) % len(TUI_ROWS)
        elif k in ("down", "j", "tab"):
            self.sel = (self.sel + 1) % len(TUI_ROWS)
        elif k == "r":
            self.next_poll = 0.0
            self.msg = "refreshing"
        elif k == "a":
            self.set_all(True)
        elif k == "x":
            self.confirm("Switch ALL outputs off? [y/N]", lambda: self.set_all(False))
        elif kind == "out" and k in (" ", "enter"):
            if not self.last:
                self.note("no reading yet", log=False)
            elif self.last["outs"][which]:
                self.confirm(f"Switch {OUTPUTS[which]} off? [y/N]", lambda: self.set_output(which, False))
            else:
                self.set_output(which, True)
        elif kind == "ch" and k in ("left", "-", "_"):
            self.step(which, -COARSE[which])
        elif kind == "ch" and k in ("right", "+", "="):
            self.step(which, COARSE[which])
        elif kind == "ch" and k in (",", "<"):
            self.step(which, -1)
        elif kind == "ch" and k in (".", ">"):
            self.step(which, 1)
        elif kind == "ch" and k in ("e", "enter"):
            self.prompt = ("edit", which, "")
            self.msg = ""
        elif kind == "ch" and k == "h":
            self.toggle_hold(which)

    def _handle_prompt(self, k):
        """confirm: y/Y runs the action, any other key cancels. edit: build the buffer; Enter
        parses it with parse_level() and writes at once (dropping any pending step), Esc cancels."""
        kind = self.prompt[0]
        if kind == "confirm":
            _, _, fn = self.prompt
            self.prompt = None
            if k in ("y", "Y"):
                fn()
            else:
                self.msg = "cancelled"
            return
        _, name, buf = self.prompt
        if k == "esc" or k == "ctrl-c":
            self.prompt = None
            self.msg = "cancelled"
        elif k == "backspace":
            self.prompt = ("edit", name, buf[:-1])
        elif k == "enter":
            self.prompt = None
            if not buf.strip():
                self.msg = "cancelled"
                return
            try:
                raw = parse_level(name, buf)
            except ValueError as e:
                self.msg = f"bad value: {e}"
                return
            self.pending.pop(name, None)
            self.apply_level(name, raw)
        elif len(k) == 1 and k.isprintable():
            self.prompt = ("edit", name, buf + k)

    # --- polling -------------------------------------------------------------------------
    def poll(self):
        """Holder.poll() with comm errors shown instead of raised; schedules the next poll.
        On error the last good reading stays on screen, marked stale."""
        try:
            self.last = self.holder.poll()
            self.err = None
        except CommError as e:
            self.holder.counts["comm"] += 1
            self.holder.event(f"comm error: {e}")
            self.hub.close()
            self.err = str(e)
        self.next_poll = time.monotonic() + self.a.interval

    # --- drawing -------------------------------------------------------------------------
    def lines(self, width, height):
        """Build the screen as a list of at most `height` lines, each clipped to `width` visible
        columns. Selectable rows are tuples ("sel", row index, text[, tail]) until _fmt() renders
        them (highlighted when selected). The event list fills leftover height; on a short
        terminal the body is cut, never the prompt/help footer."""
        s, h = self.last, self.holder
        up = int(time.time() - h.started)
        L = [f"{BOLD}SV241 Pro @ {self.hub.port or '?'}{RST}   {time.strftime('%Y-%m-%d %H:%M:%S')}   "
             f"poll {self.a.interval:g} s   up {up // 3600:02d}:{up % 3600 // 60:02d}:{up % 60:02d}"]
        if self.err:
            L.append(f"{RED}!! {self.err}; retrying{RST}" + ("   (values below are stale)" if s else ""))
        L.append("")
        L.append(f"{DIM}  {'Output':<9}{'State':<7}  (switching not yet verified on hardware){RST}")
        for i, n in enumerate(OUTPUTS):
            st = "—" if not s else ("ON" if s["outs"][i] else "off")
            L.append(("sel", i, f"  {n:<9}{st:<7}"))
        L.append("")
        L.append(f"{DIM}  {'Channel':<9}{'Device':>10}{'Raw':>6}  {'Hold':>10}   Status{RST}")
        for name in CHANNELS:
            dev = s["dev"][name] if s else None
            cell = fmt_level(name, dev)
            if name in self.pending:
                p = self.pending[name][0]
                cell = f"-> {fmt_level(name, p)}"
                dev = p
            raw = "" if dev is None else str(dev)
            hold = fmt_level(name, h.targets[name]) if name in h.targets else "—"
            status = h.status[name] if s else ""
            if name == "pwm1" and dev is not None and dev > PWM1_SOFT_MAX:
                status += f"{YEL}  >80 %: cycles at cutoff edge, 70-80 % suggested{RST}"
            if name == "adj":
                status += f"{DIM}  (volts approximate){RST}"
            L.append(("sel", len(OUTPUTS) + list(CHANNELS).index(name),
                      f"  {name.upper():<9}{cell:>10}{raw:>6}  {hold:>10}   ", status))
        if s:
            L.append("")
            L.append(f"{DIM}  {'Sensor':<14}{'Now':>10}{'Min':>11}{'Max':>11}{'Avg':>11}{RST}")
            for key, label, unit, spec in SENSOR_ROWS:
                st = h.stats[key]
                c = lambda v: (spec.format(v) + " " + unit) if v is not None else "n/a"
                L.append(f"  {label:<14}{c(s[key]):>10}{c(st.min):>11}{c(st.max):>11}{c(st.avg):>11}")
            L.append(f"  Energy {h.energy_wh:.2f} Wh since start")
            lim, lens = s["pwm1_cutoff_c"], s["lens_c"]
            if lim is not None and lens is not None:
                m = lim - lens
                L.append(f"  PWM1 cutoff {lim:.2f} °C: lens {abs(m):.2f} °C {'below' if m >= 0 else 'ABOVE'}"
                         + (f"{RED}  -> firmware holds PWM1 at 0{RST}" if m < 0 else ""))
            elif lens is None:
                L.append("  Lens probe not connected: PWM1 cutoff cannot trigger")
        # footer: message/prompt + key help, events fill whatever height is left
        foot = [""]
        if self.prompt and self.prompt[0] == "edit":
            _, name, buf = self.prompt
            unit = "volts" if name == "adj" else "percent"
            foot.append(f"{BOLD}{name.upper()} new value ({unit}, or raw:N), Enter to send, Esc to cancel: {RST}{buf}_")
        elif self.prompt:
            foot.append(f"{BOLD}{YEL}{self.prompt[1]}{RST}")
        else:
            foot.append(self.msg)
        foot.extend(f"{DIM}{k}{RST}" for k in KEY_HELP)
        room = height - len(L) - len(foot) - 2
        if room > 0:
            ev = list(h.events)[-room:]
            L.append("")
            L.append(f"{DIM}  Events{RST}")
            L.extend("  " + e for e in ev)
        L = L[:max(0, height - len(foot))] + foot   # short terminal: cut the body, never the prompt
        return [self._fmt(x, width) for x in L][-height:]

    def _fmt(self, x, width):
        """Render one lines() entry; a selected row gets reverse video and a '>' marker."""
        if isinstance(x, str):
            return clip(x, width)
        _, idx, text, *extra = x
        tail = extra[0] if extra else ""
        if idx == self.sel:
            return REV + ">" + clip(text, width - 1)[1:] + RST + clip(tail, width - visible_len(text))
        return clip(text + tail, width)

    def render(self):
        """Repaint in place: cursor home, each line erased-then-written, clear below. No full
        clear per frame, so the display doesn't flicker."""
        size = shutil.get_terminal_size((100, 40))
        body = "".join("\033[K" + ln + RST + "\n" for ln in self.lines(size.columns, size.lines))
        self.out.write("\033[H" + body.rstrip("\n") + "\033[J")
        self.out.flush()

    # --- main loop -----------------------------------------------------------------------
    def run(self):
        """Main loop until q / Ctrl+C: poll on schedule, flush debounced writes, read keys
        (short waits so debounce and redraws stay timely), redraw when something changed and at
        least once a second for the clock. Uses the alternate screen; SIGTERM/SIGHUP are turned
        into KeyboardInterrupt so the terminal is always restored. Pending writes are flushed on
        exit; the CSV is closed."""
        h = self.holder
        if self.a.hold_current:
            _, dev = self.hub.state()
            for name in CHANNELS:
                h.targets.setdefault(name, dev[name])
        desc = ", ".join(f"{k.upper()}={fmt_level(k, v)}" for k, v in h.targets.items()) or "none"
        h.event(f"TUI started; held: {desc}")
        self.out.write("\033[?1049h\033[?25l\033[2J")      # alternate screen, hide cursor
        last_draw = 0.0
        old_sig = {}

        def stop(*_):
            raise KeyboardInterrupt
        for name in ("SIGTERM", "SIGHUP"):        # window closed / kill: still restore the tty
            if hasattr(signal, name):
                try:
                    old_sig[name] = signal.signal(getattr(signal, name), stop)
                except (ValueError, OSError):    # not the main thread
                    pass
        try:
            with self.keys:
                while not self.quit:
                    dirty = False
                    if time.monotonic() >= self.next_poll:
                        self.poll()
                        dirty = True
                    if self.pending:
                        n = len(self.pending)
                        self.flush_pending()
                        dirty = dirty or len(self.pending) != n
                    if dirty or time.monotonic() - last_draw >= 1.0:
                        self.render()
                        last_draw = time.monotonic()
                    wait = min(0.1 if self.pending else 0.25, max(0.0, self.next_poll - time.monotonic()))
                    for k in self.keys.read(wait):
                        self.handle(k)
                        dirty = True
                        if self.quit:
                            break
                    if dirty and not self.quit:
                        self.render()
                        last_draw = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.flush_pending(force=True)
            except Exception:
                pass
            self.out.write(RST + "\033[?25h\033[?1049l")
            self.out.flush()
            for name, old in old_sig.items():
                signal.signal(getattr(signal, name), old)
            if h.csv:
                h.csv_f.close()


ANSI_RE = re.compile(r"\033\[[0-9;?]*[A-Za-z]")   # CSI escape sequences (colours, cursor moves)


def visible_len(s):
    """Length of s as displayed, ignoring ANSI escape sequences."""
    return len(ANSI_RE.sub("", s))


def clip(s, width):
    """Cut s to `width` visible characters, keeping escape codes intact."""
    if width <= 0:
        return ""
    if visible_len(s) <= width:
        return s
    out, n, i = [], 0, 0
    while i < len(s) and n < width:
        m = ANSI_RE.match(s, i)
        if m:
            out.append(m.group())
            i = m.end()
            continue
        out.append(s[i])
        n += 1
        i += 1
    return "".join(out) + RST


def cmd_tui(hub, a):
    """tui: interactive mode. Needs a real console on stdin and stdout (refuses pipes/mintty)."""
    if a.interval < 1:
        print("error: --interval must be at least 1 s", file=sys.stderr)
        return 2
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        print("error: tui needs an interactive console (cmd, PowerShell or Windows Terminal; "
              "in Git Bash/mintty run it via 'winpty python sv241.py tui')", file=sys.stderr)
        return 2
    if os.name == "nt":
        os.system("")   # enables ANSI escape handling in the Windows console
    Tui(a, hub).run()
    return 0


# ------------------------------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------------------------------
def build_parser():
    """argparse tree: global --port/--timeout, then one subcommand (see COMMANDS)."""
    ap = argparse.ArgumentParser(prog="sv241", description="Control and monitor an SVBONY SV241 Pro. "
                                 "With no command, runs 'hold'.")
    ap.add_argument("--port", help="COMx, /dev/ttyUSBx (Linux) or /dev/cu.usbserial-x / "
                    "/dev/cu.wchusbserialx (macOS); default: first CH340 found")
    ap.add_argument("--timeout", type=float, default=0.4, help="reply timeout in s (default 0.4)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("status", help="outputs, PWM/ADJ levels and sensors")
    p.add_argument("--json", action="store_true")
    p.add_argument("--no-sensors", action="store_true")
    p = sub.add_parser("sensors", help="sensor readings only")
    p.add_argument("--json", action="store_true")
    for n in ("on", "off"):
        p = sub.add_parser(n, help=f"switch outputs {n}: dc1..dc5, usbc, usba, or all")
        p.add_argument("outputs", nargs="+")
    p = sub.add_parser("set", help="set adj (volts), pwm1/pwm2 (percent), or raw:N")
    p.add_argument("channel", type=str.lower, choices=sorted(CHANNELS))
    p.add_argument("level")
    p = sub.add_parser("raw", help="send cmd [idx [val]] as hex, print the reply")
    p.add_argument("bytes", nargs="+")
    sub.add_parser("ports", help="list serial ports")
    p = sub.add_parser("hold", help="monitor and keep ADJ/PWM at targets (default)")
    p.add_argument("--adj", type=arg_level("adj"), help="ADJ OUT target: volts, or raw:N")
    p.add_argument("--pwm1", type=arg_level("pwm1"), help="PWM1 target: percent, or raw:N")
    p.add_argument("--pwm2", type=arg_level("pwm2"), help="PWM2 target: percent, or raw:N")
    p.add_argument("--hold-current", action="store_true",
                   help="use the hub's current values as targets for channels not given explicitly")
    p.add_argument("--interval", type=float, default=5.0,
                   help="seconds between polls (default 5; the firmware refreshes sensors every 5 s)")
    p.add_argument("--csv", help="append every poll to this CSV file")
    p.add_argument("--events", type=int, default=10, help="recent events shown (default 10)")
    p.add_argument("--no-clear", action="store_true", help="no refreshing display; print events only")
    p = sub.add_parser("tui", help="interactive live display: switch outputs, change levels with keys")
    p.add_argument("--adj", type=arg_level("adj"), help="hold ADJ OUT at this value from the start")
    p.add_argument("--pwm1", type=arg_level("pwm1"), help="hold PWM1 at this value from the start")
    p.add_argument("--pwm2", type=arg_level("pwm2"), help="hold PWM2 at this value from the start")
    p.add_argument("--hold-current", action="store_true", help="start with all three channels held at current values")
    p.add_argument("--interval", type=float, default=2.0, help="seconds between polls (default 2)")
    p.add_argument("--csv", help="append every poll to this CSV file")
    p.add_argument("--events", type=int, default=50, help="event history kept (default 50)")
    p.set_defaults(no_clear=False)
    return ap


def insert_default_command(argv):
    """Old SVBONY-hold.py command lines have no command word: slot 'hold' in after the globals.
    Global options may also appear among hold options (e.g. --pwm1 40 --port COM6), so move them
    to the front."""
    argv = list(argv)
    glob, rest, i = [], [], 0
    while i < len(argv):
        t = argv[i]
        if t in ("--port", "--timeout") and i + 1 < len(argv):
            glob += argv[i:i + 2]
            i += 2
            continue
        if t.startswith("--port=") or t.startswith("--timeout="):
            glob.append(t)
        else:
            rest.append(t)
        i += 1
    if rest and (rest[0] in COMMANDS or rest[0] in ("-h", "--help")):
        return glob + rest
    return glob + ["hold"] + rest


def main(argv=None, hub_factory=None):
    """CLI entry point; returns the exit code. Arguments are validated before the port is
    opened, so a typo never touches the hub. hub_factory replaces Hub (tests/fake_hub.py)."""
    argv = sys.argv[1:] if argv is None else argv
    a = build_parser().parse_args(insert_default_command(argv))
    try:
        if a.cmd == "ports":
            try:
                return cmd_ports(a)
            except CommError as e:
                print(f"error: {e}", file=sys.stderr)
                return 1
        if a.cmd == "set":
            parse_level(a.channel, a.level)          # validate before touching the port
        if a.cmd in ("on", "off") and [t.lower() for t in a.outputs] != ["all"]:
            parse_outputs(a.outputs)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    hub = (hub_factory or Hub)(a.port, a.timeout)
    handler = {"status": cmd_status, "sensors": cmd_sensors, "on": cmd_switch, "off": cmd_switch,
               "set": cmd_set, "raw": cmd_raw, "hold": cmd_hold, "tui": cmd_tui}[a.cmd]
    try:
        if a.cmd in ("hold", "tui"):
            return handler(hub, a)               # hold manages (re)opening itself
        with hub:
            return handler(hub, a)
    except CommError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except Refused as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 2
    finally:
        hub.close()


if __name__ == "__main__":
    if os.name == "nt" and not have_pyserial():
        sys.exit("pyserial missing: pip install pyserial (macOS/Linux work without it)")
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")   # "°" prints as garbage in Git Bash otherwise
        except Exception:
            pass
    sys.exit(main())
