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
    python sv241.py cycle usba             # USB-A off 3 s, back on; report USB devices lost/regained
    python sv241.py cycle dc2 usbc --off-time 5 --watch 20
    python sv241.py off usba --usb-window 8   # watch host USB for 8 s after switching

Host USB monitoring (no serial port needed, so it runs alongside NINA):

    python sv241.py usb                    # USB device tree; SV241 hub, its CH340, devices behind it
    python sv241.py usb --watch            # log connects/disconnects behind the SV241 hub
    python sv241.py usb --watch --all      # ... of every USB device on the PC

Hold mode (long-running monitor; the default when no command is given, so the old
SVBONY-hold.py command lines still work unchanged):

    python sv241.py --port COM6 --pwm1 40 --pwm2 40 --adj 12
    python sv241.py hold --pwm1 raw:100
    python sv241.py --hold-current            # adopt the hub's current values as targets
    python sv241.py                           # monitor only, enforce nothing
    python sv241.py --pwm1 40 --csv hub.csv   # also log every poll to CSV
    python sv241.py --pwm1 40 --no-clear      # plain scrolling event log (for services)
    python sv241.py --pwm1 40 --usb-watch     # also log USB connects/disconnects behind the hub

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

Global options go before the command (or anywhere in a hold command line): --port COM6,
--timeout 0.4, --debug FILE, --usb-hub SPEC.

One-shot exit codes: 0 success, 1 comm error, 2 hub did not take the value (e.g. PWM1 refused
by the firmware dew cutoff) or bad argument (all arguments, including `raw` hex bytes, are
checked before the port is opened); `cycle` also 3 (a USB device did not come back) and 130
(Ctrl+C; outputs are switched back on first). Settings persist on the hub (it saves them to flash).

Debug log (--debug FILE, any command)
-------------------------------------
Appends a JSONL trace, one JSON object per line, flushed per line ('-' = stderr, refused for tui).
Every record has ts (local ISO 8601 time with ms and UTC offset), mono (monotonic seconds, for
intervals) and ev (type). Types and their main fields:
  start         argv, cmd, platform, python, pyserial version, pid       exit   rc or exception
  port_open     port, auto, backend (pyserial/termios)                   port_close   port
  xfer          tx, rx (hex), expect, got (short = timeout), ms          xfer_error   tx, error
  state         outputs {DC1..USB-A: bool}, adj, pwm1, pwm2 (raw)        bad_reply    cmd, rx, attempt
  sensors       raw {"03".."07": value/100, offsets in}, derived readings (input_v, lens_c, ...)
  ack           idx, target, val, result (ok/refused/noack)              verify  target, want, got, ok
  unsolicited   hex, text, banner (true = boot banner, hub rebooted)
  comm_error / refused   where, error                                    event   msg (hold/TUI events)
  key           key, row, prompt (TUI)                                   cycle   outputs, off_time, watch
  usb_baseline  scope, count, devices    usb_add / usb_remove  dev, below_hub, mark, since_mark_s
  usb_hub       hubs, bridge, note       usb_mark  label (a switch)      usb_list / usb_error
Analyse with e.g. jq: jq -c 'select(.ev=="usb_remove")' trace.jsonl, or pandas.read_json(lines=True).

Host USB monitoring
-------------------
See the "Host-side USB monitoring" section below for how the SV241's built-in hub is found (the
CH340 bridge's parent hub, plus the SuperSpeed companion of a USB 3 hub) and the per-OS
backends. Monitoring polls the OS device list and diffs it, so a timing is "seen by" the first
poll after the OS (de)enumerated the device: one-shot windows poll every 0.2 s, `usb --watch`
every 0.5 s, hold/TUI --usb-watch every --usb-interval (1 s). Where it is used:
  on/off   watch for --usb-window s after switching (default 5 s when usbc/usba/all is switched)
  cycle    off, --off-time s, on, --watch s; prints each change with its delay after the switch,
           then a gone/back summary per device (all host USB devices, those behind the hub tagged)
  usb      tree view, or --watch for a continuous log of devices behind the hub (--all: all)
  hold/tui --usb-watch adds USB changes to the event list (and CSV/debug log); in the TUI they
           are timed from the last output switch
The hub identification is inferred from the OS device tree and NOT yet checked on the real box:
run `sv241.py usb` once and confirm the marked hub is the SV241's (--usb-hub to override).

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
  value = x / 100; temperatures and RH carry a +255 offset. (The old script's big-endian decode
  reads high by b5/100; with its empirical -255.5 / -254 offsets that came to about +0.6 °C and
  +2.2 %RH.) The firmware's upper two bytes are not true base-255 digits,
  but they are 0 for every value this script reads (x < 65025), so the decode is exact here.
  DTR/RTS must be low before the port opens, or the ESP32 resets. The hub sends nothing
  unprompted except its "SV241Pro ..." boot banner, so seeing that means it rebooted.

Hardware status (2026-11-21, COM6): reads, PWM2 write/read-back, hold mode, base-255 decode,
04 = lens / 05 = ambient and the PWM1 cutoff formula are confirmed. DC/USB output switching
(on/off, all on/off, cycle) follows the firmware disassembly but is NOT yet tested on hardware.
Host USB monitoring is tested against simulated device trees only (sysfs, CfgMgr32, ioreg).

Known unknowns: the exact ADJ OUT volts-per-raw (vendor app's 15.3 V at raw 253 is used; the
firmware only sets a 40 kHz PWM duty, so the real scale depends on the board's regulator), and
the scale of cmd 02 (INA219 power, unused here). Use raw:N when the ADJ voltage matters.

Code layout (sections in file order)
------------------------------------
  Debug log            DebugLog, DBG (the process-wide --debug JSONL writer)
  Protocol helpers     frame(), checksum_ok(), base255(), dew_point(), cutoff_limit()
  Port discovery       list_serial_ports(), pick_port(), PosixSerial (stdlib termios backend)
  Serial link          Hub: open/verify/close, xfer(), state()/sensors() reads, verified writes
  Host USB monitoring  UsbDev, linux_usb_snapshot / parse_ioreg / _win_walk + win_build,
                       find_sv241_hubs(), usb_subtree(), UsbWatcher, usb_cycle_report()
  Parsing/formatting   parse_level() (V / % / raw:N -> raw), fmt_level(), parse_outputs()
  One-shot commands    cmd_status, cmd_sensors, cmd_switch, cmd_cycle, cmd_set, cmd_raw,
                       cmd_ports, cmd_usb
  Hold mode            Stat, Holder (poll, enforce, cutoff tracking, CSV, render), cmd_hold
  TUI mode             Keys (raw key reader), Tui (keyboard actions on top of Holder), cmd_tui
  CLI                  build_parser(), insert_default_command(), main() (debug log), _run()
tests/fake_hub.py simulates the hub for tests/test_sv241.py (python -m unittest discover -s tests).

Requires Python 3.8+. Windows needs pyserial (pip install pyserial); on macOS and Linux it is
optional: without it a built-in termios backend is used and ports are found via /dev + sysfs.
USB monitoring is stdlib too (ctypes CfgMgr32 on Windows, sysfs on Linux, ioreg on macOS).
Not affiliated with SVBONY.
"""
import argparse
import collections
import csv
import datetime
import errno
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
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
INDEX_NAMES = {**dict(enumerate(OUTPUTS)), **{v: k for k, v in CHANNELS.items()}}   # idx -> name (debug log)
# Levels are raw 0..255 (firmware LEDC duty = round(raw/255 * 1023)). The vendor app calls 253
# "100 %" and "15.3 V"; those are app conventions, kept so values match what users see there.
RAW_FULL = 253           # vendor app's 100 % / 15.3 V point
RAW_MAX = 255            # firmware accepts the full byte range
ADJ_VMAX = 15.3          # assumed ADJ OUT volts at RAW_FULL (linear scale; unverified on hardware)
INA_SCALE = 15.57        # vendor app's INA219 current correction (implies a non-default board shunt)
PWR_OFFSET_MW = 269.39   # vendor app's fixed power offset (mW)
COMMANDS = ("status", "sensors", "on", "off", "set", "raw", "ports", "usb", "cycle", "hold", "tui")
GLOBAL_OPTS = ("--port", "--timeout", "--debug", "--usb-hub")   # each takes one value


class CommError(Exception):
    """Serial-level failure: port missing/busy, I/O error, or no valid reply. Exit code 1."""


class Refused(Exception):
    """The hub answered but did not take a write (read-back mismatch or refusal ack). Exit code 2."""


# ------------------------------------------------------------------------------------------
# Debug log (--debug FILE)
# ------------------------------------------------------------------------------------------
class DebugLog:
    """Structured, timestamped trace for offline analysis: one JSON object per line (JSONL).

    Every record carries
        ts    local wall-clock time, ISO 8601 with milliseconds and UTC offset
        mono  time.monotonic() in seconds: use it for intervals (immune to clock changes)
        ev    record type (listed under "Debug log" in the module header and the README)
    plus fields specific to the type. Off by default: log() then returns at once, and the hot
    path (Hub.xfer) checks `on` before formatting hex. Thread-safe (the USB watcher thread logs
    too). Each line is flushed as it is written, so the file is complete even if the process is
    killed. A write error (disk full, file closed) turns logging off with one warning on stderr
    rather than breaking the command being traced."""

    def __init__(self):
        self.on = False
        self.f = None
        self.path = None
        self._lock = threading.Lock()

    def open(self, path):
        """Start logging to `path` (appended to; "-" = stderr). Raises OSError if it can't open."""
        self.f = sys.stderr if path == "-" else open(path, "a", encoding="utf-8")
        self.path = path
        self.on = True

    def log(self, ev, **fields):
        """Write one record {ts, mono, ev, **fields}; values json can't encode go through str()."""
        if not self.on:
            return
        rec = {"ts": datetime.datetime.now().astimezone().isoformat(timespec="milliseconds"),
               "mono": round(time.monotonic(), 4), "ev": ev}
        rec.update(fields)
        line = json.dumps(rec, ensure_ascii=False, default=str)
        with self._lock:
            if not self.on:                       # closed by another thread meanwhile
                return
            try:
                self.f.write(line + "\n")
                self.f.flush()
            except (OSError, ValueError) as e:
                self.on = False
                print(f"warning: debug log {self.path} stopped: {e}", file=sys.stderr)

    def close(self):
        """Stop logging and close the file (never stderr). Safe to call repeatedly."""
        with self._lock:
            self.on = False
            if self.f is not None and self.f is not sys.stderr:
                try:
                    self.f.close()
                except OSError:
                    pass
            self.f = None


DBG = DebugLog()   # process-wide; main() opens it for --debug, everything else just calls DBG.log


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
    available without pyserial/ioreg, so only cu.wchusbserial* (a name only WCH's driver uses)
    counts as CH340; cu.usbserial-*/cu.usbmodem* are listed as unidentified USB serial."""
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
        """Open and configure `port`; raises CommError (busy, permission, not a tty, bad baud).

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
            speed = getattr(termios, f"B{baudrate}", None)
            if speed is None:
                raise CommError(f"baud rate {baudrate} not supported by termios on this OS")
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
        the overall reply timeout. After opening: 0.3 s settle, then check for a boot banner.
        Debug records: port_open on success, comm_error (where=open) on failure."""
        if self.ser:
            return self
        try:
            self._open()
        except CommError as e:
            DBG.log("comm_error", where="open", port=self.port, error=str(e))
            raise
        DBG.log("port_open", port=self.port, auto=self.port_arg is None,
                backend="pyserial" if have_pyserial() else "termios")
        return self

    def _open(self):
        """open() without the logging: pick the port, open it, settle, scan, verify a guess."""
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
            DBG.log("port_close", port=self.port)

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def _scan_unsolicited(self):
        """Consume any bytes nobody asked for. The hub only sends unprompted output at boot
        ("SV241Pro V1.3 ..."), so finding that text means it reset; latch reboot_seen.
        An I/O error here closes the port and raises CommError (xfer passes it through)."""
        try:
            n = getattr(self.ser, "in_waiting", 0)
            data = self.ser.read(n) if n else b""
            if data:
                DBG.log("unsolicited", port=self.port, hex=data.hex(" "),
                        text=data.decode("ascii", "replace"), banner=b"SV241Pro" in data)
            if b"SV241Pro" in data:
                self.reboot_seen = True
        except Exception as e:                    # port vanished between open and first read
            self.close()
            raise CommError(f"serial I/O failed on {self.port}: {e}")

    def xfer(self, tx, expect):
        """Send frame tx, return up to `expect` reply bytes (fewer on timeout; caller validates).

        Half-duplex: input is flushed first so a late reply to an earlier request can't be taken
        for this one. Any exception closes the port and becomes CommError.
        Debug records: one `xfer` per exchange (tx/rx hex, expected and received length, elapsed
        ms; a short rx means a timeout), or `xfer_error`."""
        self.open()
        t0 = time.monotonic()
        try:
            self._scan_unsolicited()   # look for a boot banner before discarding the buffer
            self.ser.reset_input_buffer()
            self.ser.write(tx)
            self.ser.flush()
            buf = bytearray()
            end = time.monotonic() + self.timeout
            while len(buf) < expect and time.monotonic() < end:
                buf += self.ser.read(expect - len(buf))
        except CommError:
            raise                                  # already worded (and port closed)
        except Exception as e:
            DBG.log("xfer_error", port=self.port, tx=bytes(tx).hex(" "), error=str(e))
            self.close()
            raise CommError(f"serial I/O failed on {self.port}: {e}")
        if DBG.on:
            DBG.log("xfer", tx=bytes(tx).hex(" "), rx=bytes(buf).hex(" "), expect=expect, got=len(buf),
                    ms=round((time.monotonic() - t0) * 1000, 1))
        return bytes(buf)

    # --- reads ---------------------------------------------------------------------------
    def state(self, tries=3):
        """CMD 08. Returns (outputs, levels): outputs = 7 bools in OUTPUTS order, levels =
        {"adj", "pwm1", "pwm2": raw 0..255}. Reply is 14 bytes: 24 0E 08 o0..o6 adj pwm1 pwm2 chk
        (outputs reported 0/1 here, although CMD 01 sets them with 00/FF). Retries, then CommError.
        Debug records: `state` (decoded), `bad_reply` per rejected attempt."""
        r = b""
        for attempt in range(1, tries + 1):
            r = self.xfer(frame(0x08), 14)
            if len(r) == 14 and r[0] == 0x24 and r[2] == 0x08 and checksum_ok(r):
                outs, dev = [v != 0 for v in r[3:10]], {"adj": r[10], "pwm1": r[11], "pwm2": r[12]}
                DBG.log("state", outputs=dict(zip(OUTPUTS, outs)), **dev)
                return outs, dev
            DBG.log("bad_reply", cmd=0x08, rx=r.hex(" "), attempt=attempt)
        raise CommError(f"bad state reply: {r.hex(' ') or 'nothing'}")

    def sensor(self, cmd, tries=2):
        """Read one sensor command (03..07). Reply is 8 bytes: 24 08 cmd b3 b4 b5 b6 chk.
        Returns the decoded value / 100, still including the +255 offset for 04/05/06, or None
        after `tries` bad replies (a missing reading is not a comm error; each is a `bad_reply`
        debug record)."""
        for attempt in range(1, tries + 1):
            r = self.xfer(frame(cmd), 8)
            if len(r) == 8 and r[0] == 0x24 and r[2] == cmd and checksum_ok(r):
                return base255(r[3:7]) / 100.0
            DBG.log("bad_reply", cmd=cmd, rx=r.hex(" "), attempt=attempt)
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
        rh = rh_raw = self.sensor(0x06)
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
        out = dict(input_v=vin, current_a=cur_a, power_w=pwr, lens_c=lens, ambient_c=amb,
                   humidity_pct=rh, dewpoint_c=dp, pwm1_cutoff_c=cutoff_limit(amb, rh))
        # Debug: raw decoded values (/100, offsets still in) next to the derived readings
        DBG.log("sensors", raw={"03": vin, "04": ta, "05": tb, "06": rh_raw, "07": cur}, **rounded(out))
        return out

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
            res = "ok" if ack[4] == 0xFF else "refused"
        else:
            res = "noack"
        DBG.log("ack", idx=idx, target=INDEX_NAMES.get(idx, "?"), val=val, result=res)
        return res

    def set_output(self, idx, on):
        """Switch output idx (0..6) on/off, verified via CMD 08; raises Refused on mismatch.
        NOT yet tested on hardware (firmware-derived). Outputs take 00/FF, never 0/1."""
        ack = self.set_raw(idx, 0xFF if on else 0x00)
        time.sleep(0.05)
        outs, _ = self.state()
        DBG.log("verify", target=OUTPUTS[idx], want=on, got=outs[idx], ok=outs[idx] == on, ack=ack)
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
        DBG.log("verify", target="all", want=on, ok=not bad, mismatched=bad)
        if bad:
            raise Refused(f"still {'off' if on else 'on'}: {', '.join(bad)}")

    def set_channel(self, name, raw):
        """Set adj/pwm1/pwm2 to raw 0..255, verified by read-back (the firmware's duty round trip
        returns the same raw value). Raises Refused, naming the PWM1 dew cutoff when it is why."""
        ack = self.set_raw(CHANNELS[name], raw)
        time.sleep(0.05)
        _, dev = self.state()
        DBG.log("verify", target=name, want=raw, got=dev[name], ok=dev[name] == raw, ack=ack)
        if dev[name] != raw:
            why = " (PWM1 dew cutoff: lens probe above limit)" if name == "pwm1" and ack == "refused" else ""
            raise Refused(f"{name.upper()} reads raw {dev[name]}, wanted {raw} (ack {ack}){why}")


# ------------------------------------------------------------------------------------------
# Host-side USB monitoring (what the PC sees appear / disappear on its USB bus)
# ------------------------------------------------------------------------------------------
# The SV241 Pro reaches the PC through one upstream USB cable into its built-in USB hub. Its
# CH340 serial bridge (the control port) is assumed to sit on that hub too, so the hub is found
# as the CH340's parent; devices plugged into the box's USB ports are the hub's other children.
# A USB 3 hub shows up as two hubs (a USB 2 half and a SuperSpeed half), so a "companion" hub is
# looked for as well. All of this is inferred from the OS device tree, not from the protocol,
# and is not yet confirmed on the real hub: `python sv241.py usb` prints what was identified so
# it can be checked, and --usb-hub overrides it.
#
# Each backend returns a snapshot {key: UsbDev} of the USB devices present right now; changes
# are found by diffing snapshots taken by polling (no OS event subscription: stdlib only).
#   Linux    /sys/bus/usb/devices (key = sysfs name such as 1-2.3; USB3 companion from the
#            port's `peer` link)
#   Windows  CfgMgr32 device tree via ctypes (key = device instance ID; companion from the
#            shared container ID)
#   macOS    `ioreg -p IOUSB -l -w0` (key = locationID; companion by heuristic only)
# Poll-based timing: a change is seen at the first poll after the OS finished (de)enumerating it,
# so timings are upper bounds with the poll interval as resolution.

UsbDev = collections.namedtuple("UsbDev", "key parent vid pid name serial ports root peer container")
UsbDev.__new__.__defaults__ = ((), False, "", "")   # ports, root, peer, container
UsbDev.__doc__ = """One USB device in a snapshot.
key       backend-specific stable id (see above); parent: key of the device it hangs off, or None
vid, pid  USB ids (None for root hubs on Windows)
name      product / friendly name; serial: iSerial ("" if none)
ports     serial port names served by this device (e.g. ("COM6",) or ("/dev/ttyUSB0",))
root      True for a host controller's root hub
peer      Linux: key of the device on the companion (USB2 <-> USB3) port, if any
container Windows: container ID (both halves of a USB3 hub share one)"""

WIN_NULL_CONTAINER = "{00000000-0000-0000-ffff-ffffffffffff}"   # "no container"/the PC itself
USB_WINDOW_S = 5.0          # default watch time after on/off of a USB output (one-shot commands)


class UsbUnavailable(Exception):
    """This OS's USB enumeration failed or is not supported (monitoring is skipped, not fatal)."""


def _read_text(path):
    """Contents of a small text file, stripped; "" if it can't be read."""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read().strip()
    except OSError:
        return ""


def _linux_port_of(name):
    """sysfs USB device name -> (hub interface dir, port dir name) of the port it is plugged into.
    1-2 -> ("1-0:1.0", "usb1-port2"); 1-2.3 -> ("1-2:1.0", "1-2-port3")."""
    bus, path = name.split("-", 1)
    if "." in path:
        hub, n = name.rsplit(".", 1)
        return f"{hub}:1.0", f"{hub}-port{n}"
    return f"{bus}-0:1.0", f"usb{bus}-port{path}"


def _linux_dev_at_port(port_dir):
    """Inverse of _linux_port_of for a port dir name: usb2-port2 -> 2-2, 2-2-port3 -> 2-2.3."""
    m = re.fullmatch(r"usb(\d+)-port(\d+)", port_dir)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    m = re.fullmatch(r"(\d+-[\d.]+)-port(\d+)", port_dir)
    return f"{m.group(1)}.{m.group(2)}" if m else ""


def linux_usb_snapshot(root="/sys/bus/usb/devices"):
    """{key: UsbDev} from sysfs. Only device nodes (names without ':'; interfaces skipped).
    Parent: 1-2.3 -> 1-2, 1-2 -> usb1 (the root hub). ports: tty nodes under the device's
    interfaces (ttyUSB* for CH340/FTDI, tty/ttyACM* for CDC-ACM). peer: the device on the
    companion port named by <hub iface>/<port>/peer (kernel 3.x+, needs ACPI/hub support; "" when
    absent). `root` is a parameter so tests can point it at a fake tree."""
    import glob
    try:
        names = os.listdir(root)
    except OSError as e:
        raise UsbUnavailable(f"can't read {root}: {e}")
    out = {}
    for name in names:
        if ":" in name:
            continue
        p = os.path.join(root, name)
        try:
            vid, pid = int(_read_text(os.path.join(p, "idVendor")), 16), int(_read_text(os.path.join(p, "idProduct")), 16)
        except ValueError:
            continue                              # not a USB device node
        is_root = name.startswith("usb")
        parent, peer = None, ""
        if not is_root:
            parent = name.rsplit(".", 1)[0] if "." in name else "usb" + name.split("-", 1)[0]
            hub_if, port = _linux_port_of(name)
            link = os.path.join(root, hub_if, port, "peer")
            if os.path.lexists(link):
                peer = _linux_dev_at_port(os.path.basename(os.path.realpath(link)))
        ttys = glob.glob(os.path.join(p, name + ":*", "tty*")) + glob.glob(os.path.join(p, name + ":*", "tty", "tty*"))
        ports = tuple(sorted({"/dev/" + os.path.basename(t) for t in ttys
                              if re.fullmatch(r"tty[A-Za-z]+\d+", os.path.basename(t))}))
        label = " ".join(x for x in (_read_text(os.path.join(p, "manufacturer")),
                                     _read_text(os.path.join(p, "product"))) if x)
        out[name] = UsbDev(name, parent, vid, pid, label or "USB device", _read_text(os.path.join(p, "serial")),
                           ports, is_root, peer)
    return out


def parse_ioreg(text):
    """{key: UsbDev} from `ioreg -p IOUSB -l -w0` output.

    Tree lines look like "  | +-o USB2.0 Hub@00100000  <class IOUSBHostDevice, ...>"; the column
    of "+-o" gives the depth. Property lines inside { } look like '"idVendor" = 6790'. Only nodes
    with idVendor are devices; a device's parent is its nearest such ancestor (controllers have no
    idVendor, so devices on a root port get parent None). Key = locationID as 0x%08x (it encodes
    the port path, so it is stable across re-plugs), else the node name."""
    nodes, stack = [], []          # stack: [(depth, node)] of the current ancestry
    cur = None
    for line in text.splitlines():
        m = re.match(r"^([ |]*)\+-o (.+?)  <class ([^,>]+)", line)
        if m:
            depth = len(m.group(1)) // 2
            while stack and stack[-1][0] >= depth:
                stack.pop()
            cur = {"name": m.group(2), "cls": m.group(3), "props": {},
                   "anc": [n for _, n in stack]}
            stack.append((depth, cur))
            nodes.append(cur)
            continue
        m = re.match(r'^[ |]*"([^"]+)" = (.*)$', line)
        if m and cur is not None:
            cur["props"][m.group(1)] = m.group(2).strip()

    def num(v):
        try:
            return int(v, 0)
        except (TypeError, ValueError):
            return None

    def text_prop(n, *names):
        for k in names:
            v = n["props"].get(k)
            if v:
                return v.strip('"')
        return ""

    def key(n):
        loc = num(n["props"].get("locationID"))
        return f"0x{loc:08x}" if loc is not None else n["name"]

    out = {}
    for n in nodes:
        vid = num(n["props"].get("idVendor"))
        if vid is None:
            continue
        par = next((a for a in reversed(n["anc"]) if num(a["props"].get("idVendor")) is not None), None)
        name = text_prop(n, "USB Product Name", "kUSBProductString") or n["name"].split("@")[0]
        root = "Root Hub" in n["name"] or "RootHub" in n["cls"]
        out[key(n)] = UsbDev(key(n), key(par) if par else None, vid, num(n["props"].get("idProduct")), name,
                             text_prop(n, "USB Serial Number", "kUSBSerialNumberString"), (), root)
    return out


def mac_usb_snapshot():
    """Run ioreg and parse it (parse_ioreg). Serial port names are not mapped on macOS."""
    try:
        r = subprocess.run(["ioreg", "-p", "IOUSB", "-l", "-w0"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as e:
        raise UsbUnavailable(f"ioreg failed: {e}")
    if r.returncode != 0:
        raise UsbUnavailable(f"ioreg exited {r.returncode}: {r.stderr.strip()[:200]}")
    return parse_ioreg(r.stdout)


_CM = None   # cfgmgr32 handle with prototypes set (Windows), created on first use


def _win_walk():
    """Yield (instance_id, parent_instance_id, friendly_name, device_desc, container_id) for every
    devnode at or below a USB node in the live Windows device tree (CfgMgr32 via ctypes).

    The tree is walked from the root with CM_Get_Child / CM_Get_Sibling; it only holds devices
    that are present, so a snapshot is exactly "what is connected now". Properties are read only
    under USB nodes (instance IDs starting USB\\), which keeps a walk to a few milliseconds.
    Registry codes: CM_DRP_DEVICEDESC 0x01, CM_DRP_FRIENDLYNAME 0x0D, CM_DRP_BASE_CONTAINERID 0x25."""
    global _CM
    import ctypes
    from ctypes import wintypes
    DI, UL = wintypes.DWORD, wintypes.ULONG
    if _CM is None:
        cm = ctypes.WinDLL("cfgmgr32")
        cm.CM_Locate_DevNodeW.argtypes = [ctypes.POINTER(DI), wintypes.LPCWSTR, UL]
        cm.CM_Get_Child.argtypes = [ctypes.POINTER(DI), DI, UL]
        cm.CM_Get_Sibling.argtypes = [ctypes.POINTER(DI), DI, UL]
        cm.CM_Get_Device_IDW.argtypes = [DI, wintypes.LPWSTR, UL, UL]
        cm.CM_Get_DevNode_Registry_PropertyW.argtypes = [DI, UL, ctypes.POINTER(UL), ctypes.c_void_p,
                                                         ctypes.POINTER(UL), UL]
        for f in (cm.CM_Locate_DevNodeW, cm.CM_Get_Child, cm.CM_Get_Sibling, cm.CM_Get_Device_IDW,
                  cm.CM_Get_DevNode_Registry_PropertyW):
            f.restype = UL                        # CONFIGRET; 0 = CR_SUCCESS
        _CM = cm
    cm = _CM
    root = DI()
    if cm.CM_Locate_DevNodeW(ctypes.byref(root), None, 0) != 0:
        raise UsbUnavailable("CM_Locate_DevNodeW failed")
    idbuf = ctypes.create_unicode_buffer(512)      # MAX_DEVICE_ID_LEN is 200
    pbuf = ctypes.create_unicode_buffer(1024)

    def dev_id(dn):
        return idbuf.value if cm.CM_Get_Device_IDW(dn, idbuf, len(idbuf), 0) == 0 else None

    def prop(dn, code):
        size, typ = UL(ctypes.sizeof(pbuf)), UL()
        if cm.CM_Get_DevNode_Registry_PropertyW(dn, code, ctypes.byref(typ), pbuf, ctypes.byref(size), 0) != 0:
            return ""
        return pbuf.value

    stack, child = [(root.value, None, False)], DI()
    while stack:
        dn, parent_id, under_usb = stack.pop()
        iid = dev_id(dn)
        if iid is None:                           # vanished mid-walk
            continue
        usb = under_usb or iid.upper().startswith("USB\\")
        if usb:
            yield iid, parent_id, prop(dn, 0x0D), prop(dn, 0x01), prop(dn, 0x25)
        if cm.CM_Get_Child(ctypes.byref(child), dn, 0) == 0:
            c = child.value
            while True:
                stack.append((c, iid, usb))
                if cm.CM_Get_Sibling(ctypes.byref(child), c, 0) != 0:
                    break
                c = child.value


def win_build(nodes):
    """{key: UsbDev} from _win_walk() tuples. Kept separate from the ctypes walk so it can be
    tested anywhere.

    Only USB *device* nodes are kept: instance IDs USB\\VID_xxxx&PID_yyyy\\<serial or port id>
    and USB\\ROOT_HUB*; interface nodes (&MI_xx) and function nodes below them (HID\\, FTDIBUS\\,
    ...) are folded into the nearest device above, contributing any "(COMn)" in their names to its
    `ports`. A device description in raw INF form ("@usb.inf,%id%;Name") is cut to Name. Parent = nearest USB device ancestor. Serial = last ID segment unless it contains '&'
    (Windows generates such IDs for devices without a serial number)."""
    nodes = list(nodes)
    info = {n[0]: n for n in nodes}

    def is_dev(iid):
        u = iid.upper()
        return u.startswith("USB\\") and "&MI_" not in u

    def owner(iid):
        seen = 0
        while iid is not None and iid in info and seen < 64:
            if is_dev(iid):
                return iid
            iid, seen = info[iid][1], seen + 1
        return None

    ports = collections.defaultdict(set)
    for iid, _par, friendly, _desc, _cont in nodes:
        o = owner(iid)
        for m in re.finditer(r"\((COM\d+)\)", friendly or ""):
            if o:
                ports[o].add(m.group(1))
    out = {}
    for iid, par, friendly, desc, cont in nodes:
        if not is_dev(iid):
            continue
        if desc.startswith("@") and ";" in desc:   # raw INF reference "@usb.inf,%id%;Generic USB Hub"
            desc = desc.rsplit(";", 1)[-1]
        m = re.search(r"VID_([0-9A-F]{4})&PID_([0-9A-F]{4})", iid, re.I)
        tail = iid.split("\\")[-1]
        out[iid] = UsbDev(iid, owner(par), int(m.group(1), 16) if m else None, int(m.group(2), 16) if m else None,
                          friendly or desc or iid, tail if "&" not in tail else "", tuple(sorted(ports[iid])),
                          "ROOT_HUB" in iid.upper(), "", (cont or "").lower())
    return out


def usb_snapshot():
    """{key: UsbDev} of every USB device the host sees now; raises UsbUnavailable."""
    if sys.platform.startswith("linux"):
        return linux_usb_snapshot()
    if sys.platform == "darwin":
        return mac_usb_snapshot()
    if os.name == "nt":
        try:
            return win_build(_win_walk())
        except (OSError, AttributeError) as e:     # cfgmgr32 missing / call failure
            raise UsbUnavailable(f"CfgMgr32 enumeration failed: {e}")
    raise UsbUnavailable(f"no USB enumeration backend for {sys.platform}")


def usb_subtree(snap, roots):
    """Keys of devices at or below any key in roots (the roots themselves included)."""
    roots, out = set(roots), set()
    for k in snap:
        c, seen = k, 0
        while c is not None and seen < 64:
            if c in roots:
                out.add(k)
                break
            d = snap.get(c)
            c, seen = (d.parent if d else None), seen + 1
    return out


def _is_hub(snap, d):
    """Heuristic: named like a hub, or has children in the snapshot."""
    return "hub" in d.name.lower() or any(x.parent == d.key for x in snap.values())


def usb_companions(snap, hub):
    """Keys of the other half of a USB3 hub (see section comment), most reliable source first:
    Linux port peer link; Windows shared container ID (ignoring the null container); else any
    other hub with the same VID on the same parent, or with both on root hubs (two halves sit on
    different root hubs on Linux/macOS). The heuristic can pick a second identical hub plugged
    in next to the SV241; --usb-hub overrides."""
    if hub.peer:
        return [hub.peer] if hub.peer in snap else []
    if hub.container and hub.container != WIN_NULL_CONTAINER:
        same = [d.key for d in snap.values() if d.key != hub.key and d.container == hub.container
                and _is_hub(snap, d) and d.parent not in usb_subtree(snap, [hub.key])]
        if same:
            return same

    def on_root(d):
        p = snap.get(d.parent)
        return d.parent is None or (p is not None and p.root)
    return [d.key for d in snap.values()
            if d.key != hub.key and not d.root and d.vid == hub.vid and _is_hub(snap, d)
            and (d.parent == hub.parent or (on_root(d) and on_root(hub)))
            and hub.key not in usb_subtree(snap, [d.key]) and d.key not in usb_subtree(snap, [hub.key])]


def _same_port(a, b):
    """Serial port names equal, ignoring case and any /dev/ prefix (COM6 == com6)."""
    return os.path.basename(str(a)).lower() == os.path.basename(str(b)).lower()


def find_sv241_hubs(snap, port=None, spec=None):
    """Identify the SV241's USB hub(s) in a snapshot. Returns (hub keys, bridge key, note).

    spec (--usb-hub): comma-separated keys or VID:PID (hex) values, taken as the hub list as-is
    (bridge None). Otherwise: the CH340 bridge serving `port` (or the only CH340, or the first of
    several, with a note), then its parent hub plus usb_companions(). If the CH340's parent is a
    host root hub, the SV241 hub can't be told from the PC's own ports, so none is returned and
    the note says so. hub keys = [] with a note when nothing fits."""
    if spec:
        keys, missing = [], []
        for item in (x.strip() for x in spec.split(",") if x.strip()):
            m = re.fullmatch(r"([0-9A-Fa-f]{4}):([0-9A-Fa-f]{4})", item)
            hits = ([d.key for d in snap.values() if (d.vid, d.pid) == (int(m.group(1), 16), int(m.group(2), 16))]
                    if m else [k for k in snap if k.lower() == item.lower()])
            keys += [k for k in hits if k not in keys]
            if not hits:
                missing.append(item)
        return keys, None, (f"--usb-hub {', '.join(missing)} not present" if missing else None)
    ch = [d for d in snap.values() if (d.vid, d.pid) == CH340]
    if not ch:
        return [], None, "no CH340 (1A86:7523) on the USB bus: SV241 not connected?"
    note = None
    if port:
        mine = [d for d in ch if any(_same_port(port, p) for p in d.ports)]
        ch = mine or ch
    if len(ch) > 1:
        ch.sort(key=lambda d: d.key)
        note = f"{len(ch)} CH340 bridges and none matched the port; using {ch[0].key} (pass --port or --usb-hub)"
    bridge = ch[0]
    hub = snap.get(bridge.parent)
    if hub is None or hub.root:
        return [], bridge.key, (f"CH340 {bridge.key} is on a host root port, so the SV241 hub could "
                                "not be identified; pass --usb-hub")
    return [hub.key] + [k for k in usb_companions(snap, hub) if k != hub.key], bridge.key, note


def usb_label(d):
    """One-line description: VID:PID name (ports) serial [key]."""
    ids = f"{d.vid:04X}:{d.pid:04X}" if d.vid is not None and d.pid is not None else "----:----"
    ports = [p for p in d.ports if p not in d.name]
    return (f"{ids} {d.name}" + (f" ({', '.join(ports)})" if ports else "")
            + (f" s/n {d.serial}" if d.serial else "") + f" [{d.key}]")


def usb_record(d):
    """UsbDev -> plain dict for the debug log (ids as hex text)."""
    r = d._asdict()
    r["vid"] = f"{d.vid:04x}" if d.vid is not None else None
    r["pid"] = f"{d.pid:04x}" if d.pid is not None else None
    r["ports"] = list(d.ports)
    return r


class UsbWatcher:
    """Poll the host's USB devices and report what appears and disappears.

    scope "hub": only devices at or below the SV241 hub(s) (find_sv241_hubs, re-resolved
    whenever a hub is missing, so unplugging and re-plugging the whole box is followed);
    scope "all": every USB device on the host, with devices below the SV241 hub tagged.

    sink(line) gets one human-readable line per change; every change is also a debug record
    (usb_add / usb_remove; usb_baseline, usb_hub, usb_mark, usb_error). mark(label) records a
    switching action: changes within MARK_WINDOW_S of it say "+x.xx s after <label>", and
    `history` keeps (monotonic time, "+"/"-", UsbDev, below_hub) for cycle reports.

    poll() can be called directly (one-shot windows) or from start()'s background thread
    (hold/TUI/usb --watch). `port` is a port name or a callable returning one (the Hub's port is
    only known once it has opened). `snapshot` defaults to usb_snapshot, looked up at call time."""

    MARK_WINDOW_S = 60.0

    def __init__(self, scope="hub", port=None, hub_spec=None, sink=None, snapshot=None):
        self.scope, self._port, self.hub_spec = scope, port, hub_spec
        self.sink = sink or (lambda line: None)
        self._snapshot = snapshot
        self.hubs, self.bridge = [], None
        self.prev = None                  # in-scope devices at the last poll
        self.prev_below = set()           # keys below the SV241 hub at the last poll
        self.mark_label, self.mark_t = None, None
        self.history = []
        self.available = True
        self._said = None                 # last hub/unavailable note, so it is said once
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    @property
    def port(self):
        return self._port() if callable(self._port) else self._port

    def _say(self, line, once=False):
        """Send a line to the sink; once=True suppresses an immediate repeat of the same line."""
        if once and line == self._said:
            return
        if once:
            self._said = line
        self.sink(line)

    def mark(self, label):
        """Note that `label` (e.g. "USB-A off") happens now; later changes are timed from it."""
        with self._lock:
            self.mark_label, self.mark_t = label, time.monotonic()
        DBG.log("usb_mark", label=label)

    def _resolve(self, snap):
        hubs, bridge, note = find_sv241_hubs(snap, self.port, self.hub_spec)
        if hubs and hubs != self.hubs:
            self._say("SV241 USB hub: " + "; ".join(usb_label(snap[h]) for h in hubs), once=True)
            DBG.log("usb_hub", hubs=[usb_record(snap[h]) for h in hubs], bridge=bridge, note=note)
        elif not hubs and self.hubs:
            self._say("SV241 USB hub is gone from the bus", once=True)
            DBG.log("usb_hub", hubs=[], bridge=bridge, note="hub gone")
        if note:
            self._say(f"note: {note}", once=True)
        self.hubs, self.bridge = hubs, bridge

    def poll(self):
        """Take one snapshot and report changes since the previous one. The first successful call
        only records the baseline. Returns [(kind, UsbDev)] with kind "+" (appeared) / "-" (gone)."""
        snap_fn = self._snapshot or usb_snapshot
        try:
            snap = snap_fn()
        except UsbUnavailable as e:
            line = f"USB monitoring unavailable: {e}"
            if line != self._said:
                DBG.log("usb_error", error=str(e))
            self._say(line, once=True)
            self.available = False
            return []
        self.available = True
        now = time.monotonic()
        with self._lock:
            if self.hub_spec or not self.hubs or any(h not in snap for h in self.hubs):
                self._resolve(snap)
            below = usb_subtree(snap, self.hubs) if self.hubs else set()
            cur = {k: snap[k] for k in (snap if self.scope == "all" else below)}
            if self.prev is None:
                self.prev, self.prev_below = cur, below
                DBG.log("usb_baseline", scope=self.scope, count=len(cur),
                        devices=[dict(usb_record(d), below_hub=d.key in below) for d in cur.values()])
                return []
            changes = ([("-", self.prev[k]) for k in sorted(self.prev) if k not in cur]
                       + [("+", cur[k]) for k in sorted(cur) if k not in self.prev])
            prev_below = self.prev_below
            self.prev, self.prev_below = cur, below
            label, t_mark = self.mark_label, self.mark_t
        for kind, d in changes:
            under = d.key in (prev_below if kind == "-" else below)
            since = now - t_mark if t_mark is not None and now - t_mark <= self.MARK_WINDOW_S else None
            self.history.append((now, kind, d, under))
            DBG.log("usb_remove" if kind == "-" else "usb_add", dev=usb_record(d), below_hub=under,
                    mark=label if since is not None else None,
                    since_mark_s=round(since, 3) if since is not None else None)
            self._say(f"USB {'connected   ' if kind == '+' else 'DISCONNECTED'} {usb_label(d)}"
                      + (" (behind SV241 hub)" if under and self.scope == "all" else "")
                      + (f"  +{since:.2f} s after {label}" if since is not None else ""))
        return changes

    def watch_for(self, seconds, step=0.2):
        """Poll every `step` s for `seconds` (foreground; one-shot commands)."""
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            time.sleep(min(step, left))
            self.poll()

    def count(self):
        """Devices currently in scope (None before the first successful poll)."""
        return None if self.prev is None else len(self.prev)

    def start(self, interval=1.0):
        """Take the baseline now, then poll every `interval` s in a daemon thread until stop().
        Unexpected exceptions in the thread are logged (usb_error) and polling continues."""
        self.poll()

        def run():
            while not self._stop.wait(interval):
                try:
                    self.poll()
                except Exception as e:            # never let the watcher kill the main program
                    DBG.log("usb_error", error=repr(e))
                    self._say(f"USB watcher error: {e!r}", once=True)
        self._thread = threading.Thread(target=run, name="usb-watch", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        """Stop the background thread (if any) and wait briefly for it."""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)


def usb_cycle_report(w, t_off, t_on, watched):
    """Summarise a cycle from w.history: for each device, when it disappeared after the off
    switch (t_off, monotonic) and when it came back after the on switch (t_on); devices that did
    not come back within `watched` s; devices that are new. Returns (lines, n_not_back)."""
    gone, back, new = {}, {}, []
    for t, kind, d, under in w.history:
        if t < t_off:
            continue
        if kind == "-" and d.key not in gone:
            gone[d.key] = (t, d, under)
        elif kind == "+":
            if d.key in gone and d.key not in back:
                back[d.key] = t
            elif d.key not in gone:
                new.append((t, d, under))
    lines, missing = [], 0
    for k, (t, d, under) in sorted(gone.items(), key=lambda x: x[1][0]):
        tag = " (behind SV241 hub)" if under else ""
        if k in back and back[k] < t_on:
            lines.append(f"  gone +{t - t_off:.2f} s after off, back +{back[k] - t_off:.2f} s after off "
                         f"(before the on switch): {usb_label(d)}{tag}")
        elif k in back:
            lines.append(f"  gone +{t - t_off:.2f} s after off, back +{back[k] - t_on:.2f} s after on: {usb_label(d)}{tag}")
        else:
            missing += 1
            lines.append(f"  gone +{t - t_off:.2f} s after off, NOT back within {watched:g} s: {usb_label(d)}{tag}")
    for t, d, under in new:
        lines.append(f"  new  +{t - t_on:.2f} s after on (not seen before): {usb_label(d)}"
                     + (" (behind SV241 hub)" if under else ""))
    if not lines:
        lines.append("  no USB devices disappeared or appeared")
    return lines, missing


def ts_print(line):
    """Print a line prefixed with HH:MM:SS.mmm (USB watcher sink for one-shot commands)."""
    now = datetime.datetime.now()
    print(f"{now:%H:%M:%S}.{now.microsecond // 1000:03d} {line}", flush=True)


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
        x *= RAW_FULL / (ADJ_VMAX if name == "adj" else 100.0)
        if not math.isfinite(x) or abs(x) > 1e6:            # 1e308 overflows to inf above
            raise ValueError(f"{name}: {s} is outside raw 0..{RAW_MAX}")
        raw = round(x)
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
# Each cmd_* handler takes (hub, args) (cmd_ports and cmd_usb: args only, no hub) and returns
# the process exit code; _run() maps CommError -> 1 and Refused -> 2 (UsbUnavailable -> 1 for
# cmd_usb) for anything they let escape.
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


USB_OUTPUTS = (5, 6)        # USB-C, USB-A: the outputs whose switching is expected to move USB devices


def switch_targets(tokens):
    """on/off/cycle output arguments -> sorted indexes; a lone "all" means all 7."""
    return list(range(len(OUTPUTS))) if [t.lower() for t in tokens] == ["all"] else parse_outputs(tokens)


def usb_window_watcher(hub, a, idxs):
    """UsbWatcher (scope "all", baseline taken, printing timestamped lines) for watching USB
    changes around an on/off, or None. Window: --usb-window if given, else USB_WINDOW_S when a USB
    output is among idxs (DC outputs can power USB devices too, but usually don't), else 0."""
    win = a.usb_window if a.usb_window is not None else (USB_WINDOW_S if set(idxs) & set(USB_OUTPUTS) else 0)
    if win <= 0:
        return None, 0
    w = UsbWatcher("all", port=lambda: hub.port or a.port, hub_spec=a.usb_hub, sink=ts_print)
    w.poll()
    return (w if w.available else None), win


def cmd_switch(hub, a):
    """on/off <outputs...|all>. Each output is switched and verified separately; failures are
    reported and the rest still attempted (rc 2 if any failed). "all" uses the firmware all-on/off
    command. With a USB watch window (see usb_window_watcher), host USB connects/disconnects are
    printed with their delay after the switch, then summarised. Untested on hardware."""
    on = a.cmd == "on"
    idxs = switch_targets(a.outputs)
    w, win = usb_window_watcher(hub, a, idxs)
    label = ("all" if [t.lower() for t in a.outputs] == ["all"]
             else "+".join(OUTPUTS[i] for i in idxs)) + (" on" if on else " off")
    if w:
        w.mark(label)
    rc = 0
    if [t.lower() for t in a.outputs] == ["all"]:
        hub.set_all(on)
        print(f"all outputs {'ON' if on else 'off'}")
    else:
        for i in idxs:
            try:
                hub.set_output(i, on)
                print(f"{OUTPUTS[i]} {'ON' if on else 'off'}")
            except Refused as e:
                print(f"FAILED: {e}", file=sys.stderr)
                rc = 2
    if w:
        print(f"watching USB for {win:g} s ...", flush=True)
        w.watch_for(win)
        n = len(w.history)
        print(f"USB: {n} change{'s' if n != 1 else ''} within {win:g} s of {label}")
    return rc


def cmd_cycle(hub, a):
    """cycle <outputs...|all> [--off-time S] [--watch S]: switch outputs off, wait, switch them
    back on, and report which host USB devices disappeared and came back, with timings.

    Only outputs that are on are cycled (an output already off stays off; a cycle never turns on
    something that was off). If all seven are on, the firmware all-off/all-on commands are used.
    The on switch is in a finally block, so Ctrl+C or an error during the off time still restores
    the outputs (retried for ~10 s across comm errors). Exit: 0, 2 if a switch was refused, 3 if
    a device that disappeared did not come back within --watch s, 130 if interrupted (Ctrl+C
    during the off time switches back on at once and skips the report).

    NOT yet tested on hardware. If the box's own CH340 were powered through a switched USB output,
    switching that output off would cut this program off from the hub with the output still off;
    the identified hub and bridge are printed first so that can be checked (`sv241.py usb`)."""
    outs, _ = hub.state()
    want = switch_targets(a.outputs)
    idxs = [i for i in want if outs[i]]
    for i in want:
        if not outs[i]:
            print(f"{OUTPUTS[i]} is off; not cycled")
    if not idxs:
        print("nothing to cycle")
        return 0
    use_all = len(idxs) == len(OUTPUTS)
    names = "all" if use_all else "+".join(OUTPUTS[i] for i in idxs)
    w = UsbWatcher("all", port=lambda: hub.port or a.port, hub_spec=a.usb_hub, sink=ts_print)
    w.poll()
    if not w.available:
        print("continuing without USB monitoring")
    DBG.log("cycle", outputs=[OUTPUTS[i] for i in idxs], off_time=a.off_time, watch=a.watch)
    rc = 0

    def switch(on):
        if use_all:
            hub.set_all(on)
        else:
            for i in idxs:
                hub.set_output(i, on)
        print(f"{names} {'ON' if on else 'off'}", flush=True)

    w.mark(f"{names} off")
    t_off = time.monotonic()
    interrupted = False
    try:
        switch(False)
        try:
            w.watch_for(a.off_time)
        except KeyboardInterrupt:
            interrupted = True
            print("interrupted: switching back on now", flush=True)
    finally:
        w.mark(f"{names} on")
        t_on = time.monotonic()
        for attempt in range(20):                 # ~10 s: ride out a USB/serial hiccup
            try:
                switch(True)
                break
            except CommError as e:
                hub.close()
                if attempt == 19:
                    print(f"error: could not switch {names} back on: {e}", file=sys.stderr)
                    raise
                time.sleep(0.5)
            except Refused as e:
                print(f"FAILED: {e}", file=sys.stderr)
                rc = 2
                break
    if interrupted:
        return 130
    w.watch_for(a.watch)
    if w.available:
        lines, missing = usb_cycle_report(w, t_off, t_on, a.watch)
        print(f"USB summary ({names} off {a.off_time:g} s, watched {a.watch:g} s after on):")
        for ln in lines:
            print(ln)
        if missing and rc == 0:
            rc = 3
    return rc


def cmd_usb(a):
    """usb [--watch [--all] [--interval S]]: host-side USB view; never opens the serial port, so
    it can run while NINA holds the hub.

    Without --watch: list every USB device as a tree, marking the SV241 hub(s), its CH340 bridge
    and everything behind the hub. With --watch: report connects/disconnects below the SV241 hub
    (every device with --all) until Ctrl+C."""
    port = a.port
    if port is None:
        try:
            port = pick_port(list_serial_ports())[0]
        except CommError:
            port = None
    if not a.watch:
        snap = usb_snapshot()
        hubs, bridge, note = find_sv241_hubs(snap, port, a.usb_hub)
        below = usb_subtree(snap, hubs)
        kids = collections.defaultdict(list)
        for d in snap.values():
            kids[d.parent if d.parent in snap else None].append(d)

        def walk(parent, depth):
            for d in sorted(kids[parent], key=lambda x: x.key):
                tag = ("   <- SV241 hub" if d.key in hubs else "   <- SV241 serial bridge" if d.key == bridge
                       else "   (behind SV241)" if d.key in below else "")
                print("  " * depth + usb_label(d) + tag)
                walk(d.key, depth + 1)
        walk(None, 0)
        if not snap:
            print("no USB devices found")
        if note:
            print(f"note: {note}")
        DBG.log("usb_list", hubs=hubs, bridge=bridge, note=note,
                devices=[dict(usb_record(d), below_hub=d.key in below) for d in snap.values()])
        return 0
    if a.interval < 0.1:
        print("error: --interval must be at least 0.1 s", file=sys.stderr)
        return 2
    w = UsbWatcher("all" if a.all else "hub", port=port, hub_spec=a.usb_hub, sink=ts_print)
    w.poll()
    if not w.available:
        return 1
    what = "USB devices" if a.all else "devices behind the SV241 hub"
    ts_print(f"watching {w.count()} {what}, polling every {a.interval:g} s (Ctrl+C to stop)")
    n, max_polls = 0, getattr(a, "max_polls", None)
    try:
        while max_polls is None or n < max_polls:
            time.sleep(a.interval)
            w.poll()
            n += 1
    except KeyboardInterrupt:
        pass
    return 0


def cmd_set(hub, a):
    """set adj|pwm1|pwm2 <level>: one verified write (Refused -> rc 2 via main())."""
    name = a.channel
    raw = parse_level(name, a.level)
    hub.set_channel(name, raw)
    print(f"{name.upper()} = {fmt_level(name, raw)} (raw {raw})")
    return 0


RAW_BYTE_RE = re.compile(r"(0[xX])?[0-9a-fA-F]{1,2}")   # one byte: 0..ff, optional 0x


def parse_raw_bytes(tokens):
    """`raw` arguments -> list of ints. Each token must be 1-2 hex digits with an optional 0x/0X
    prefix (strict: no sign, whitespace or "_", which int(t, 16) would accept); at most 3
    (cmd, idx, val). Raises ValueError otherwise, so main() rejects bad input with
    exit 2 before the port is opened (frame() would otherwise silently mask values > 0xFF)."""
    if len(tokens) > 3:
        raise ValueError(f"raw takes at most 3 bytes (cmd idx val), got {len(tokens)}")
    vals = []
    for t in tokens:
        if not RAW_BYTE_RE.fullmatch(t):
            raise ValueError(f"raw: '{t}' is not a hex byte 00..ff")
        vals.append(int(t, 16))
    return vals


def cmd_raw(hub, a):
    """raw <cmd> [idx [val]] (hex): send one frame, print whatever arrives within the timeout.
    Debug aid: the command byte is not checked against the known set, and the reply is shown
    even if malformed. Missing idx/val default to 0."""
    vals = parse_raw_bytes(a.bytes)
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
        ids = f"{vid:04X}:{pid:04X}" if vid is not None and pid is not None else "????:????"
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
        self._ev_lock = threading.Lock()  # event() is also called from the USB watcher thread
        self.usb = None                   # UsbWatcher when --usb-watch (see start_usb)
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
        """Record a timestamped event; with --no-clear also print it (service-friendly log).
        Every event is also an `event` debug record. Thread-safe (USB watcher thread)."""
        with self._ev_lock:
            line = f"{time.strftime('%H:%M:%S')} {msg}"
            self.events.append(line)
            self._last_event = msg
            DBG.log("event", msg=msg)
            if self.a.no_clear:
                print(line, flush=True)

    def start_usb(self, port=None):
        """With --usb-watch, start a background UsbWatcher whose lines become events: scope
        "hub" (devices behind the SV241 hub), or "all" with --usb-all. No-op otherwise."""
        if not getattr(self.a, "usb_watch", False):
            return
        self.usb = UsbWatcher("all" if getattr(self.a, "usb_all", False) else "hub",
                              port=port or (lambda: self.hub.port or self.a.port),
                              hub_spec=getattr(self.a, "usb_hub", None), sink=self.event)
        self.usb.start(getattr(self.a, "usb_interval", 1.0))

    def stop_usb(self):
        if self.usb:
            self.usb.stop()

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
        if self.usb:
            n = self.usb.count()
            L.append(f"\nUSB  {'n/a' if n is None else n} device(s) "
                     + ("on the host" if self.usb.scope == "all" else "behind the SV241 hub")
                     + ("" if self.usb.available else "   (monitoring unavailable)"))
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
        --interval seconds until Ctrl+C (or max_polls, for tests). Starts/stops the USB watcher
        (--usb-watch) around the loop and closes the CSV on exit."""
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
            self.start_usb()
            self._loop(max_polls)
        finally:
            self.stop_usb()
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
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            except (termios.error, OSError):      # tty hung up
                pass

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
        """Switch one output (verified by Hub.set_output). Untested on hardware. With
        --usb-watch, USB changes after it are timed from this switch."""
        if self.holder.usb:
            self.holder.usb.mark(f"{OUTPUTS[i]} {'on' if on else 'off'}")
        self.act(lambda: self.hub.set_output(i, on), f"{OUTPUTS[i]} switched {'ON' if on else 'off'}")

    def set_all(self, on):
        """Firmware all-on / all-off. Untested on hardware. Marks the USB watcher like set_output."""
        if self.holder.usb:
            self.holder.usb.mark(f"all {'on' if on else 'off'}")
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
            except Refused as e:
                # A held PWM1 refused by the dew cutoff keeps its new target (the holder restores
                # it when the cutoff clears); any other refusal reverts to the previous target.
                # firmware only refuses PWM1 for the cutoff; set_channel says so in the message
                cutoff = name == "pwm1" and (self.holder.in_cutoff or "dew cutoff" in str(e))
                if prev is not None and not cutoff:
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
        """Dispatch one key token. An open prompt (confirm / value entry) takes all keys.
        Each key is a `key` debug record (with the selected row and whether a prompt was open)."""
        DBG.log("key", key=k, row="%s:%s" % self.row(), prompt=self.prompt[0] if self.prompt else None)
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
        if h.usb:
            n = h.usb.count()
            L.append("")
            L.append(f"  USB {'n/a' if n is None else n} device(s) "
                     + ("on the host" if h.usb.scope == "all" else "behind the SV241 hub")
                     + ("" if h.usb.available else f"{RED}  monitoring unavailable{RST}"))
        # footer: message/prompt + key help, events fill whatever height is left
        foot = ["", *(f"{DIM}{k}{RST}" for k in KEY_HELP)]   # prompt goes last: kept on tiny screens
        if self.prompt and self.prompt[0] == "edit":
            _, name, buf = self.prompt
            unit = "volts" if name == "adj" else "percent"
            foot.append(f"{BOLD}{name.upper()} new value ({unit}, or raw:N), Enter to send, Esc to cancel: {RST}{buf}_")
        elif self.prompt:
            foot.append(f"{BOLD}{YEL}{self.prompt[1]}{RST}")
        else:
            foot.append(self.msg)
        room = height - len(L) - len(foot) - 2
        if room > 0:
            ev = list(h.events)[-room:]
            L.append("")
            L.append(f"{DIM}  Events{RST}")
            L.extend("  " + e for e in ev)
        L = L[:max(0, height - len(foot))] + foot   # short terminal: cut the body, never the prompt
        return [self._fmt(x, width) for x in L][-height:] if height > 0 else []

    def _fmt(self, x, width):
        """Render one lines() entry; a selected row gets reverse video and a '>' marker."""
        if isinstance(x, str):
            return clip(x, width)
        _, idx, text, *extra = x
        tail = extra[0] if extra else ""
        if idx == self.sel:
            return clip(REV + ">" + text[1:] + RST + tail, width)    # same width rule as below
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
        h.start_usb()
        self.out.write("\033[?1049h\033[?25l\033[2J")      # alternate screen, hide cursor
        last_draw = 0.0
        old_sig = {}

        def stop(*_):
            raise KeyboardInterrupt
        try:
            for name in ("SIGTERM", "SIGHUP"):    # window closed / kill: still restore the tty
                if hasattr(signal, name):
                    try:
                        old_sig[name] = signal.signal(getattr(signal, name), stop)
                    except (ValueError, OSError):  # not the main thread
                        pass
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
            try:
                self.out.write(RST + "\033[?25h\033[?1049l")
                self.out.flush()
            except OSError:                       # terminal already gone (SIGHUP)
                pass
            for name, old in old_sig.items():
                if old is not None:
                    try:
                        signal.signal(getattr(signal, name), old)
                    except (ValueError, OSError, TypeError):
                        pass
            h.stop_usb()
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
    ap.add_argument("--debug", metavar="FILE",
                    help="append a structured JSONL trace (serial frames, decodes, writes, events, USB "
                         "changes) to FILE; '-' = stderr (not with tui)")
    ap.add_argument("--usb-hub", metavar="SPEC",
                    help="SV241 USB hub override for USB monitoring: comma-separated device keys or "
                         "VID:PID (see 'usb'); default: the CH340 bridge's parent hub + USB3 companion")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("status", help="outputs, PWM/ADJ levels and sensors")
    p.add_argument("--json", action="store_true")
    p.add_argument("--no-sensors", action="store_true")
    p = sub.add_parser("sensors", help="sensor readings only")
    p.add_argument("--json", action="store_true")
    for n in ("on", "off"):
        p = sub.add_parser(n, help=f"switch outputs {n}: dc1..dc5, usbc, usba, or all")
        p.add_argument("outputs", nargs="+")
        p.add_argument("--usb-window", type=float, metavar="S",
                       help=f"watch host USB changes for S s after switching (default {USB_WINDOW_S:g} "
                            "when usbc/usba/all is switched, else 0 = off)")
    p = sub.add_parser("cycle", help="switch outputs off, wait, back on; report USB devices lost/regained")
    p.add_argument("outputs", nargs="+")
    p.add_argument("--off-time", type=float, default=3.0, help="seconds to stay off (default 3)")
    p.add_argument("--watch", type=float, default=10.0,
                   help="seconds to watch for devices returning after switching on (default 10)")
    p = sub.add_parser("usb", help="host USB devices behind the SV241 hub (no serial port used)")
    p.add_argument("--watch", action="store_true", help="report connects/disconnects until Ctrl+C")
    p.add_argument("--all", action="store_true", help="with --watch: every USB device, not only the SV241's")
    p.add_argument("--interval", type=float, default=0.5, help="poll interval for --watch (default 0.5 s)")
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
    p.add_argument("--usb-watch", action="store_true",
                   help="log host USB connects/disconnects behind the SV241 hub as events")
    p.add_argument("--usb-all", action="store_true", help="with --usb-watch: every USB device on the host")
    p.add_argument("--usb-interval", type=float, default=1.0, help="USB poll interval (default 1 s)")
    p = sub.add_parser("tui", help="interactive live display: switch outputs, change levels with keys")
    p.add_argument("--adj", type=arg_level("adj"), help="hold ADJ OUT at this value from the start")
    p.add_argument("--pwm1", type=arg_level("pwm1"), help="hold PWM1 at this value from the start")
    p.add_argument("--pwm2", type=arg_level("pwm2"), help="hold PWM2 at this value from the start")
    p.add_argument("--hold-current", action="store_true", help="start with all three channels held at current values")
    p.add_argument("--interval", type=float, default=2.0, help="seconds between polls (default 2)")
    p.add_argument("--csv", help="append every poll to this CSV file")
    p.add_argument("--events", type=int, default=50, help="event history kept (default 50)")
    p.add_argument("--usb-watch", action="store_true",
                   help="log host USB connects/disconnects behind the SV241 hub as events")
    p.add_argument("--usb-all", action="store_true", help="with --usb-watch: every USB device on the host")
    p.add_argument("--usb-interval", type=float, default=1.0, help="USB poll interval (default 1 s)")
    p.set_defaults(no_clear=False)
    return ap


def insert_default_command(argv):
    """Old SVBONY-hold.py command lines have no command word: slot 'hold' in after the globals.
    Global options (GLOBAL_OPTS) may also appear among hold options (e.g. --pwm1 40 --port COM6),
    so move them to the front."""
    argv = list(argv)
    glob, rest, i = [], [], 0
    while i < len(argv):
        t = argv[i]
        if t in GLOBAL_OPTS and i + 1 < len(argv):
            glob += argv[i:i + 2]
            i += 2
            continue
        if t.startswith(tuple(o + "=" for o in GLOBAL_OPTS)):
            glob.append(t)
        else:
            rest.append(t)
        i += 1
    if rest and (rest[0] in COMMANDS or rest[0] in ("-h", "--help")):
        return glob + rest
    return glob + ["hold"] + rest


def main(argv=None, hub_factory=None):
    """CLI entry point; returns the exit code. `set` levels, `on`/`off`/`cycle` names, `raw` hex
    bytes and time values are validated before the port is opened (exit 2), so a typo never
    touches the hub. hub_factory replaces Hub (tests/fake_hub.py).

    --debug FILE opens the JSONL trace (DBG) for the whole run: a `start` record (argv, platform,
    Python and pyserial versions) first and an `exit` record (rc, or the exception) last."""
    argv = sys.argv[1:] if argv is None else argv
    a = build_parser().parse_args(insert_default_command(argv))
    if a.debug:
        if a.debug == "-" and a.cmd == "tui":
            print("error: --debug - (stderr) would garble the tui screen; give a file name", file=sys.stderr)
            return 2
        try:
            DBG.open(a.debug)
        except OSError as e:
            print(f"error: can't open debug log {a.debug}: {e}", file=sys.stderr)
            return 2
        try:
            import serial
            pyser = getattr(serial, "__version__", "?")
        except ImportError:
            pyser = None
        DBG.log("start", argv=list(argv), cmd=a.cmd, platform=sys.platform, os_name=os.name,
                python=sys.version.split()[0], pyserial=pyser, pid=os.getpid())
    rc = None
    try:
        rc = _run(a, hub_factory)
        return rc
    except BaseException as e:
        DBG.log("exit", rc=rc, exception=repr(e))
        rc = "logged"
        raise
    finally:
        if rc != "logged":
            DBG.log("exit", rc=rc)
        DBG.close()


def _run(a, hub_factory):
    """main() after argument parsing and debug-log setup: validate, then dispatch."""
    try:
        if a.cmd == "ports":
            try:
                return cmd_ports(a)
            except CommError as e:
                print(f"error: {e}", file=sys.stderr)
                return 1
        if a.cmd == "usb":
            try:
                return cmd_usb(a)
            except UsbUnavailable as e:
                print(f"error: USB enumeration unavailable: {e}", file=sys.stderr)
                return 1
        if a.cmd == "set":
            parse_level(a.channel, a.level)          # validate before touching the port
        if a.cmd in ("on", "off", "cycle"):
            switch_targets(a.outputs)
        if a.cmd in ("on", "off") and a.usb_window is not None and not 0 <= a.usb_window <= 600:
            raise ValueError("--usb-window must be 0..600 s")
        if a.cmd == "cycle" and not (0 <= a.off_time <= 600 and 0 <= a.watch <= 600):
            raise ValueError("--off-time and --watch must be 0..600 s")
        if a.cmd in ("hold", "tui") and a.usb_interval < 0.1:
            raise ValueError("--usb-interval must be at least 0.1 s")
        if a.cmd == "raw":
            parse_raw_bytes(a.bytes)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    hub = (hub_factory or Hub)(a.port, a.timeout)
    handler = {"status": cmd_status, "sensors": cmd_sensors, "on": cmd_switch, "off": cmd_switch,
               "cycle": cmd_cycle, "set": cmd_set, "raw": cmd_raw, "hold": cmd_hold, "tui": cmd_tui}[a.cmd]
    try:
        if a.cmd in ("hold", "tui"):
            return handler(hub, a)               # hold manages (re)opening itself
        with hub:
            return handler(hub, a)
    except CommError as e:
        DBG.log("comm_error", where=a.cmd, error=str(e))
        print(f"error: {e}", file=sys.stderr)
        return 1
    except Refused as e:
        DBG.log("refused", where=a.cmd, error=str(e))
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
