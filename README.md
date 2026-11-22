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

Global options: `--port COM6`, `--timeout 0.4` (reply timeout, s), `--debug FILE`,
`--debug-text FILE`, `--no-usb`, `--usb-hub SPEC`. They may go anywhere on the command line
(`status --port COM6` works); they are moved in front of the command before parsing.

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

Exit codes: `0` ok, `1` comm error (for `usb`: USB enumeration failed), `2` refused by hub
(e.g. PWM1 during dew cutoff) or bad argument (levels, output names, `raw` bytes and times are
checked before the port is opened); `cycle` also `3` (a USB device that disappeared did not come
back within `--watch`) and `130` (Ctrl+C during the off time; every output was switched back
on). `2` wins over `130` if an output could not be switched back on. Settings persist on the
hub (firmware auto-saves to flash every 5 s).

`on`/`off`/`all`, the FE/FF frames and the host USB watch behind `cycle` are confirmed on hardware (Windows,
2026-11-22); `cycle` itself and the TUI output keys have not been run on the hub (see Status).

### Debug log (`--debug FILE`, `--debug-text FILE`)

Works with every command, including `hold` and `tui`. Appends a JSONL trace (one JSON object
per line, flushed per line, so it survives a kill): every serial exchange with tx/rx hex and
round-trip ms, decoded state and sensor values (raw and derived), write acks and read-back
verifications, hold/TUI events, TUI key presses, comm errors, boot banners, and USB changes.
`--debug -` writes to stderr (not allowed for `tui`).

`--debug-text FILE` writes the same records as plain lines for reading, alone or alongside the
JSONL file (use a different file):

```
06:20:09.994  xfer         tx 24 06 01 01 00 2c  ->  rx 24 06 01 01 ff 2c  6/6 B  0.8 ms
06:20:10.044  state        outputs DC1=on DC2=off DC3=on DC4=on DC5=on USB-C=on USB-A=on  adj=198 pwm1=253 pwm2=253
06:20:10.044  verify       target=DC2 want=False got=False ok=True ack=ok
06:20:11.310  usb_remove   - 03c3:120e ZWO ASI120MM [1-2.3] behind SV241 hub  +0.42 s after USB-A off
```

Either debug option **turns on host USB monitoring** (see below) unless `--no-usb` is given:
`hold`/`tui` behave as with `--usb-watch` (changes also show in the event list), and the other
serial commands run a silent watcher of the devices behind the SV241 hub for their duration
(0.5 s polls; baseline, changes and a final poll go to the debug log only). `cycle` and
`on`/`off` with a USB window already watch in the foreground. `--no-usb` only cancels this
implicit watching; explicit `--usb-watch`, `--usb-window` and `cycle` still watch.

Each record has `ts` (local ISO 8601, ms, UTC offset), `mono` (monotonic s; use it for
intervals) and `ev`:

| `ev` | Fields |
| --- | --- |
| `start` / `exit` | argv, cmd, platform, os_name, Python/pyserial versions, pid / rc or exception |
| `port_open` / `port_close` | port, auto-detected?, backend (pyserial / termios) |
| `xfer` / `xfer_error` | tx, rx (hex), expect, got (short = timeout), ms / error |
| `state` | outputs `{DC1..USB-A: bool}`, adj, pwm1, pwm2 (raw) |
| `sensors` | raw `{"03".."07"}` (value/100, offsets included) + derived readings |
| `ack`, `verify` | idx, target, val, result (ok/refused/noack) / target, want, got, ok, ack (all-on/off: target `all`, want, ok, `mismatched` list) |
| `bad_reply`, `unsolicited` | rejected reply / unprompted bytes (`banner: true` = hub rebooted) |
| `comm_error`, `refused`, `event`, `key` | errors (`where`), hold/TUI event text, TUI key + selected row |
| `cycle`, `cycle_restore_failed` | cycle outputs, off_time, watch / outputs still off after the restore |
| `usb_baseline`, `usb_add`, `usb_remove`, `usb_hub`, `usb_mark`, `usb_list`, `usb_error` | USB monitoring (below) |

```
python sv241.py --debug trace.jsonl tui          # USB changes included automatically
python sv241.py --debug trace.jsonl --debug-text trace.log --pwm1 75
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
python sv241.py --usb-watch --pwm1 75   # hold mode, USB changes in the event list/CSV (implied by --debug)
python sv241.py tui --usb-watch         # TUI; changes are timed from the last output switch
python sv241.py cycle usba              # off 3 s, on, watch 10 s, gone/back summary per device
```

`usb` never opens the serial port, so it runs while NINA holds the hub. `on`/`off` watch host
USB for `--usb-window` s after switching (default 5 s when `usbc`, `usba` or `all` is switched,
else off). Timings are "seen by" the first poll after the OS finished (de)enumerating (0.2 s
polls in one-shot windows, `--interval` 0.5 s for `usb --watch`, `--usb-interval` 1 s in
hold/TUI, 0.5 s for the silent `--debug` watcher), so read them as upper bounds. `cycle` only cycles outputs that are on and switches them back on even
after Ctrl+C or an error (each output retried separately for ~10 s; any still off is named on
stderr and the exit code is 2).

**Hub identification on the real box** (Windows, 2026-11-22): the CfgMgr32 walk works (22
devices in 13–16 ms, keys stable between walks) and finds the CH340 on COM6. Its parent, the
hub marked as the SV241's, is a `1A40:0101` (Terminus FE1.1s, USB 2) hub. The USB-C output's
devices hang off it, but the USB-A output's devices do not. On the test box they sit under a VIA
VL817 USB 3 hub (`2109:2817` USB 2 half, `2109:0817` SuperSpeed half, `2109:8817` billboard),
which is the FE1.1s's parent on a PC root port. That VL817 is probably the SV241's own upstream
hub; this is unconfirmed, because an external hub between PC and box would look the same. Until
that is settled the default still marks only the CH340's parent. Pass
`--usb-hub 2109:2817,2109:0817` to watch the USB-A devices too. Use `--usb-hub` likewise if the
wrong hub is marked or the CH340 hangs directly off a PC port (comma-separated keys or `VID:PID`
values from the `usb` listing).

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
this is untested on the hub (it is only connected to Windows, where pyserial lowers both before
opening and three opens caused no reset); a reset shows up as a "hub REBOOTED" event.

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

`--interval` must be at least 1 s; polling faster than 5 s mostly re-reads the same sensor
sample. Other hold flags: `--events N` (recent events shown, default 10), `--usb-watch` /
`--usb-all` / `--usb-interval S` (see Host USB monitoring; implied by `--debug`).

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
| Up/Down, `j`/`k`, Tab | select row (7 outputs, ADJ, PWM1, PWM2); Tab moves down |
| Space / Enter (output row) | toggle; switching **off** asks `y/N` |
| Left/Right, `-`/`+` (also `_`/`=`) | step ADJ ~0.5 V (8 raw, up to raw 255), PWM 5 % (13 raw, capped at 100 % = raw 253) |
| `,` / `.` (also `<`/`>`) | step 1 raw |
| `e` / Enter (channel row) | type a value: `12` (V), `40` (%), `raw:120`; Esc cancels |
| `h` | hold/release the selected channel (re-pushed each poll, like hold mode) |
| `a` / `x` | all outputs on / off (`x` asks `y/N`) |
| `r` / `q` | poll now / quit (pending step is sent first; settings stay as they are) |

Step keys are batched: one write 0.6 s after the last press. Every write is read back; refusals
(e.g. PWM1 during dew cutoff) and comm errors show on the message line and in the event list,
and the display keeps running. Rows flag the open items below: output keys not yet tried on the
hub, ADJ volts approximate. TUI flags: `--adj/--pwm1/--pwm2` (held
from the start), `--hold-current`, `--interval` (default 2 s, min 1), `--csv`, `--events N`
(history kept, default 50), `--usb-watch`/`--usb-all`/`--usb-interval`.

Needs a real console: cmd, PowerShell or Windows Terminal. In Git Bash/mintty use
`winpty python sv241.py tui` (native Windows Python sees no TTY there and the TUI refuses to start).

### PWM1 dew cutoff

Firmware forces PWM1 to 0 and refuses writes while lens probe > limit (ambient > 15 °C: dew
point + 7.5 °C; 0–15 °C: dew point + 10 °C; < 0 °C: 5 °C fixed), then restores the previous level
itself (the level saved when the cutoff started, not a value written during it). No hysteresis,
so the cutoff acts as a bang-bang thermostat at the limit whenever the heater can push the lens
past it: on hardware 70, 75 and 80 % all cycled (see Status). Hold mode detects this (refusal ack + its own
copy of the calculation) and reports it rather than treating it as a lost setting. PWM2 has no
cutoff.

### Power readings

Vendor formula: corrected INA219 current × input V + 0.269 W. Sensors sample every 5 s and each
INA219 read is a sub-ms snapshot while heaters switch at 10 Hz, so single readings see heaters
fully on or off. Use hold mode's Avg / Wh, not Max.

## Status

**Tested on hardware** (Windows, COM6, Python 3.13, pyserial 3.5):

- 2026-11-20/21: all reads, PWM2 write/restore, hold mode, checksums, `0x04` = lens probe,
  base-255 sensor decode, PWM1 cutoff formula.
- 2026-11-22 (bench script; results in the project thread):
  - Output switching. `01` on idx 0–6 is acked `FF` and reads back as written, about 57 ms for
    write plus verify. The all-off and all-on frames are acked `24 06 FF FF FF 2A` and
    `24 06 FE FE FF 28`, and read back all off / all on.
  - Physical mapping. idx 4 = DC5 was seen on the box. DC1–DC4 had no visible load, so only
    their read-back is confirmed.
  - USB-C (idx 5) cuts the devices on the FE1.1s hub. USB-A (idx 6) cuts the devices under the
    VL817 hub (see Host USB monitoring).
  - The serial link stays up with either USB output off; the CH340 never dropped.
  - Re-enumeration. Devices are gone at the first 0.2 s poll; downstream hubs and their children
    take up to about 2.6 s. Devices are back 0.5–1.8 s after switching on.
  - The Windows CfgMgr32 USB walk works. Hub identification works, but see the open item below.
  - Port open via pyserial (DTR/RTS low before open) does not reset the hub: 3 opens, no boot
    banner.
  - The `msvcrt` key decoding is correct for injected console keys: arrows, Enter, Esc,
    Backspace, Space, `q , . + - j k`.
  - PWM1 cutoff details: writes are refused during the cutoff; on release the firmware restores
    the level saved at cutoff entry.

**Untested on hardware** (simulator / synthetic data only):

- `cycle` as a command and the TUI output keys (the frames and USB watching they use are
  confirmed).
- ADJ OUT volts scale (`set adj <V>`, `--adj <V>`).
- The TUI on the hub. Only its Windows key decoding is confirmed; the rest ran only against
  `tests/fake_hub.py` on Linux.
- macOS/Linux termios backend and the POSIX DTR/RTS open: the hub is only connected to Windows.
  These ran only over a pty against the simulator on Linux; not on a real CH340 or on macOS.
- Host USB monitoring on Linux/macOS (simulated sysfs / ioreg only).

Open:

- **DC1–DC4 physical mapping** (idx 0–3) not seen on the box: nothing visible was plugged in
  during the test. idx 4 = DC5 matched.
- **SV241 hub boundary.** The USB-A output's devices hang off a VL817 hub above the
  CH340's FE1.1s parent, so the default identification misses them. It is unclear whether that
  VL817 is inside the box (see Host USB monitoring); `--usb-hub 2109:2817,2109:0817` until
  then.
- **ADJ OUT volts per raw step unknown.** `set adj <V>` uses the vendor app's 15.3 V @ raw 253
  linear assumption; ADJ is a 40 kHz PWM into a regulator, so the real curve needs a meter. Use
  `raw:N` when it matters.
- **PWM1 cycles at 70–80 % as well as at 100 %.** The earlier advice that ~70–80 % avoids
  cycling is withdrawn. On 2026-11-22 (ambient 16.7 °C, RH 79–81 %, limit = dew point + 7.5 ≈
  20.6–21.1 °C) each of 70, 75 and 80 % held for 10 min spent 53–59 % of the 5 s samples in
  cutoff. That was 4–5 episodes per 10 min, about 75 s on and 95 s cut, with the lens within
  ±0.5 °C of the limit. In those conditions the duty only changes how fast the lens reaches the
  limit. Whether a lower duty stays clear in colder or drier air is untested.
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
non-obvious protocol details (checksum, base-255 decode, PWM1 cutoff, DTR/RTS, the unmeasured
ADJ scale). Keep them accurate in every change: update a comment whenever the code it describes
changes, and update `docs/PROTOCOL.md` when protocol knowledge changes.
