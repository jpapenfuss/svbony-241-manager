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

Global options go before the command: `--port COM6`, `--timeout 0.4`, `--debug FILE`,
`--usb-hub SPEC`.

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
python sv241.py cycle usba [--off-time 3] [--watch 10]   # off, wait, on; report USB devices lost/regained
python sv241.py off usba --usb-window 8          # watch host USB for 8 s after switching
```

Exit codes: `0` ok, `1` comm error, `2` refused by hub (e.g. PWM1 during dew cutoff) or bad
argument (checked before the port is opened); `cycle` also `3` (a USB device that disappeared
did not come back within `--watch`) and `130` (Ctrl+C during the off time: outputs are switched
back on first). Settings persist on the hub (firmware auto-saves to flash every 5 s).

### Debug log (`--debug FILE`)

Works with every command, including `hold` and `tui`. Appends a JSONL trace (one JSON object
per line, flushed per line, so it survives a kill): every serial exchange with tx/rx hex and
round-trip ms, decoded state and sensor values (raw and derived), write acks and read-back
verifications, hold/TUI events, TUI key presses, comm errors, boot banners, and USB changes.
`--debug -` writes to stderr (not allowed for `tui`).

Each record has `ts` (local ISO 8601, ms, UTC offset), `mono` (monotonic s; use it for
intervals) and `ev`:

| `ev` | Fields |
| --- | --- |
| `start` / `exit` | argv, platform, Python/pyserial versions / rc or exception |
| `port_open` / `port_close` | port, auto-detected?, backend (pyserial / termios) |
| `xfer` / `xfer_error` | tx, rx (hex), expect, got (short = timeout), ms / error |
| `state` | outputs `{DC1..USB-A: bool}`, adj, pwm1, pwm2 (raw) |
| `sensors` | raw `{"03".."07"}` (value/100, offsets included) + derived readings |
| `ack`, `verify` | write target, value, ack result / wanted vs read-back |
| `bad_reply`, `unsolicited` | rejected reply / unprompted bytes (`banner: true` = hub rebooted) |
| `comm_error`, `refused`, `event`, `key`, `cycle` | errors, hold/TUI events, TUI keys, cycle parameters |
| `usb_baseline`, `usb_add`, `usb_remove`, `usb_hub`, `usb_mark`, `usb_list`, `usb_error` | USB monitoring (below) |

```
python sv241.py --debug trace.jsonl tui
jq -c 'select(.ev=="xfer" and .got < .expect)' trace.jsonl          # timeouts
jq -c 'select(.ev=="usb_remove") | [.ts, .dev.name, .since_mark_s]' trace.jsonl
python -c "import pandas as pd; print(pd.read_json('trace.jsonl', lines=True).ev.value_counts())"
```

### Host USB monitoring

The SV241 Pro's upstream USB-C feeds a built-in hub; its CH340 control port is assumed to sit
on that hub, so the script finds the hub as **the CH340's parent** in the OS device tree, plus
the SuperSpeed **companion** a USB 3 hub also shows up as. Devices behind those hubs are "behind
the SV241". Stdlib only; snapshots of the OS device list are diffed by polling:

| OS | Source | Device key | USB 3 companion from |
| --- | --- | --- | --- |
| Windows | CfgMgr32 device tree (ctypes) | instance ID | shared container ID |
| Linux | `/sys/bus/usb/devices` | sysfs name (`1-2.3`) | port `peer` link |
| macOS | `ioreg -p IOUSB -l` | locationID | same-VID hub heuristic |

```
python sv241.py usb                     # tree; marks SV241 hub, its CH340 and devices behind it
python sv241.py usb --watch             # continuous connect/disconnect log behind the hub
python sv241.py usb --watch --all       # every USB device on the PC
python sv241.py --usb-watch --pwm1 75   # hold mode, USB changes in the event list/CSV
python sv241.py tui --usb-watch         # TUI; changes are timed from the last output switch
python sv241.py cycle usba              # off 3 s, on, watch 10 s, gone/back summary per device
```

`usb` never opens the serial port, so it runs while NINA holds the hub. `on`/`off` watch host
USB for `--usb-window` s after switching (default 5 s when `usbc`, `usba` or `all` is switched,
else off). Timings are "seen by" the first poll after the OS finished (de)enumerating (0.2 s
polls in one-shot windows, 0.5 s for `usb --watch`, `--usb-interval` 1 s in hold/TUI), so read
them as upper bounds. `cycle` only cycles outputs that are on and switches them back on even
after Ctrl+C or an error.

**Check the hub identification once on the real box** (`python sv241.py usb`): it is inferred,
not yet verified. If the wrong hub is marked, or the CH340 turns out to hang directly off a PC
port, pass `--usb-hub` with keys or `VID:PID` values from that listing (comma-separated).

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

Other hold flags: `--events N` (recent events shown), `--usb-watch` / `--usb-all` /
`--usb-interval S` (see Host USB monitoring).

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
| Left/Right, `-`/`+` | step ADJ ~0.5 V (8 raw), PWM 5 % (13 raw, capped at 100 %) |
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

- **DC/USB output switching untested on hardware** (`on`/`off`, all-on/off, `cycle`). Frames
  are firmware-verified; physical mapping idx 0 = DC1 etc. not yet bench-checked. Before the
  first USB `cycle`, run `usb` and confirm the CH340 sits behind the SV241 hub rather than being
  powered through a switched USB output (if it were, switching that output off would cut the
  script off with the output left off).
- **Host USB monitoring** is tested only against simulated device trees (fake sysfs, CfgMgr32
  node lists, sample ioreg output); the ctypes CfgMgr32 walk has not run on Windows yet.
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
| `sv241.py` | Controller: one-shot commands, hold mode, TUI, debug log, host USB monitoring; layout map in its module docstring |
| `svbony-probe.py` | Checksum experiment (0xFF vs 0x00 for sum = 255) on a PWM1 write; hard-coded COM6 |
| `docs/PROTOCOL.md` | Wire protocol, encoding, firmware behaviour |
| `tests/fake_hub.py` | Software SV241 Pro (frames, checksum, sensors, PWM1 cutoff); `python tests/fake_hub.py tui` runs any sv241.py command against it |
| `tests/test_sv241.py` | Simulator tests incl. termios backend over a pty, debug log, USB backends/watcher/cycle on fake device trees: `python -m unittest discover -s tests` (no pyserial needed) |

## Contributing

The Python files carry detailed comment blocks: a module header (purpose, usage, protocol
summary pointing at `docs/PROTOCOL.md`), docstrings on classes and functions, and inline notes on
non-obvious protocol details (checksum, base-255 decode, PWM1 cutoff, DTR/RTS, untested output
switching). Keep them accurate in every change: update a comment whenever the code it describes
changes, and update `docs/PROTOCOL.md` when protocol knowledge changes.
