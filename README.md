# svbony-241-manager

Command-line and interactive (TUI) control and monitoring for the **SVBONY SV241 Pro** power hub (firmware V1.3) over its
USB serial port, without the vendor app. Switch DC/USB outputs, set dew heaters (PWM1/PWM2) and ADJ
OUT, read input V/current/power, lens and ambient temperature, humidity; or run a long-lived hold
loop that keeps heater/ADJ targets enforced.

Protocol reference: [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Requirements

- Python 3.8+ (tested on 3.13).
- **Windows:** `pip install pyserial` (tested 3.5).
- **macOS / Linux:** no dependencies. pyserial is used if installed; otherwise a built-in
  `termios` backend drives the port and auto-detect scans `/dev` (see below).
- Hub enumerates as a CH340 (`1A86:7523`), 115200 8N1. Auto-detected if `--port` is omitted;
  `python sv241.py ports` lists ports with CH340s flagged. On the NINA PC it is `COM6`.
- **Exclusive port.** Disconnect the hub in NINA, close the vendor app and any other copy of
  this script first. Opening the port with DTR/RTS high resets the ESP32; the script keeps them low.

## Usage

Global options go before the command: `--port COM6`, `--timeout 0.4`.

### One-shot commands

Open the port, do one thing, verify by read-back, exit.

```
python sv241.py status [--json] [--no-sensors]   # outputs, ADJ/PWM levels, sensors
python sv241.py sensors [--json]
python sv241.py on  dc1 dc3 usbc                 # dc1..dc5 (or 1..5), usbc, usba
python sv241.py off dc2
python sv241.py on all | off all                 # firmware all-on / all-off
python sv241.py set pwm1 40                      # percent
python sv241.py set pwm2 raw:120                 # raw 0..255
python sv241.py set adj 12                       # volts (approximate, see open items)
python sv241.py raw 08                           # send cmd [idx [val]] hex, print reply
python sv241.py ports
```

Exit codes: `0` ok, `1` comm error, `2` refused by hub (e.g. PWM1 during dew cutoff) or bad
argument. Settings persist on the hub (firmware auto-saves to flash every 5 s).

### macOS / Linux

Same commands, including `tui` (ANSI + `termios`; `curses` is not used so all platforms share
one renderer). Port names and auto-detect:

| OS | Device | Auto-detect |
| --- | --- | --- |
| Linux | `/dev/ttyUSB0` (kernel `ch341` driver) | `/dev/ttyUSB*`, `/dev/ttyACM*`; VID:PID read from sysfs |
| macOS 10.14+ | `/dev/cu.usbserial-XXXX` (Apple CH34x driver) | `/dev/cu.*`; without pyserial VID:PID is unknown, so a lone `usbserial`/`usbmodem` port is used as a guess |
| macOS + WCH driver | `/dev/cu.wchusbserialXXXX` | treated as CH340 by name |

`python sv241.py ports` shows what it found and what auto-detect would pick. Use `cu.*`, not
`tty.*`, on macOS (`tty.*` blocks on open waiting for carrier).

Linux gotchas: add yourself to `dialout` (`uucp` on Arch) and log in again, or open fails with
permission denied. On Ubuntu 22.04+ **brltty** claims `1A86:7523` and the port vanishes right
after plugging in: `sudo apt remove brltty` (or mask its udev rule).

DTR/RTS: POSIX kernels raise both lines on open (pyserial does too); the script drops them
immediately and clears `HUPCL`. Both together don't trip the ESP32 auto-reset circuit, but
this is untested on the hub; a reset shows up as a "hub REBOOTED" event.

### Hold mode

Long-running monitor; the default when no command is given (old `SVBONY-hold.py` command lines
work unchanged). Polls state and sensors every `--interval` s (default 5), shows a refreshing
display, re-pushes any target the hub no longer reports, and reads each push back. Ctrl+C exits
and leaves settings as they are.

```
python sv241.py --port COM6 --pwm1 40 --pwm2 40 --adj 12
python sv241.py hold --pwm1 raw:100
python sv241.py --hold-current            # adopt current hub values as targets
python sv241.py                           # monitor only
python sv241.py --pwm1 40 --csv hub.csv   # log every poll
python sv241.py --pwm1 40 --no-clear      # plain event log (services/redirects)
```

Other hold flags: `--events N` (recent events shown).

### Interactive mode (TUI)

Live display plus keyboard control. Same polling, stats, CSV and enforcement as hold mode;
defaults to a 2 s poll. Stdlib only (ANSI escapes + `msvcrt` on Windows, `termios`/`select` on
POSIX); no `curses`/`windows-curses` needed.

```
python sv241.py --port COM6 tui
python sv241.py tui --pwm1 75 --interval 1   # start with PWM1 held at 75 %
python sv241.py tui --hold-current --csv hub.csv
```

| Key | Action |
| --- | --- |
| Up/Down, `j`/`k` | select row (7 outputs, ADJ, PWM1, PWM2) |
| Space / Enter (output row) | toggle; switching **off** asks `y/N` |
| Left/Right, `-`/`+` | step ADJ 0.5 V, PWM 5 % |
| `,` / `.` | step 1 raw |
| `e` / Enter (channel row) | type a value: `12` (V), `40` (%), `raw:120`; Esc cancels |
| `h` | hold/release the selected channel (re-pushed each poll, like hold mode) |
| `a` / `x` | all outputs on / off (`x` asks `y/N`) |
| `r` / `q` | poll now / quit (pending step is sent first; settings stay as they are) |

Step keys are batched: one write 0.6 s after the last press. Every write is read back; refusals
(e.g. PWM1 during dew cutoff) and comm errors show on the message line and in the event list,
and the display keeps running. Rows flag the open items below: output switching unverified, ADJ
volts approximate, PWM1 above 80 % (cutoff cycling).

Needs a real console: cmd, PowerShell or Windows Terminal. In Git Bash/mintty use
`winpty python sv241.py tui` (native Windows Python sees no TTY there and the TUI refuses to start).

### PWM1 dew cutoff

Firmware forces PWM1 to 0 and refuses writes while lens probe > limit (ambient > 15 °C: dew
point + 7.5 °C; 0–15 °C: dew point + 10 °C; < 0 °C: 5 °C fixed), then restores the previous level
itself. No hysteresis, so a strong heater cycles. Hold mode detects this (refusal ack + its own
copy of the calculation) and reports it rather than treating it as a lost setting. PWM2 has no
cutoff.

### Power readings

Vendor formula: corrected INA219 current × input V + 0.269 W. Sensors sample every 5 s and each
INA219 read is a sub-ms snapshot while heaters switch at 10 Hz, so single readings see heaters
fully on or off. Use hold mode's Avg / Wh, not Max.

## Status

Hardware-tested 2026-11-21 on COM6: all reads, PWM2 write/restore, hold mode, checksums,
`0x04` = lens probe, base-255 sensor decode, PWM1 cutoff formula.

TUI mode (`tui`): tested only against `tests/fake_hub.py` on Linux, not yet on the hub or on
Windows (the `msvcrt` key path is untested).

macOS/Linux: termios backend tested over a pty against the simulator on Linux. Not yet run on a
real CH340 or on macOS.

Open:

- **DC/USB output switching untested on hardware** (`on`/`off`, all-on/off). Frames are
  firmware-verified; physical mapping idx 0 = DC1 etc. not yet bench-checked.
- **ADJ OUT volts per raw step unknown.** `set adj <V>` uses the vendor app's 15.3 V @ raw 253
  linear assumption; ADJ is a 40 kHz PWM into a regulator, so the real curve needs a meter. Use
  `raw:N` when it matters.
- **PWM1 at 100 % cycles** at the cutoff edge (lens sits ~0.2–0.6 °C under the limit). ~70–80 %
  is recommended.
- `0x02` power scale unclear (raw ≈ 1/12.5 of vendor-formula W); the script computes power
  from V × I instead.

## Files

| File | Purpose |
| --- | --- |
| `sv241.py` | Controller: one-shot commands + hold mode |
| `svbony-probe.py` | Checksum experiment (0xFF vs 0x00 for sum = 255) on a PWM1 write; hard-coded COM6 |
| `docs/PROTOCOL.md` | Wire protocol, encoding, firmware behaviour |
| `tests/fake_hub.py` | Software SV241 Pro (frames, checksum, sensors, PWM1 cutoff); `python tests/fake_hub.py tui` runs any sv241.py command against it |
| `tests/test_sv241.py` | Simulator tests incl. termios backend over a pty: `python -m unittest discover -s tests` (no pyserial needed) |
