# SV241 Pro control protocol

ESP32 (firmware V1.3, banner `SV241Pro V1.3 2025-12-18`) behind a CH340 USB-serial bridge,
115200 8N1, 6-byte binary command frames. No HID, network or Bluetooth path; NINA/ASCOM drivers use
the same serial port.

Tags: **[F]** confirmed in V1.3 firmware (strings, or disassembly of the serial parser at
`0x400d307c` and reply builder at `0x400d22f4`). **[H]** confirmed on hardware (COM6, Windows, 2026-11-20/21/22).
**[C]** observed device behaviour from earlier client code, not provable from code. **[V]** vendor
Readme. **[A]** vendor app convention. **[I]** inferred.

Sources: `main.ino.bin` (V1.3; SHA-256 `8cf41c0c…`, ESP32 Arduino core 2.0.14 / ESP-IDF 4.4.6;
the vendor package is labelled V1.2 20251220), vendor `Readme.txt` changelog, vendor app behaviour,
the original hold script (superseded by `sv241.py`), `svbony-probe.py`.

## Transport

| Item | Value | Tag |
| --- | --- | --- |
| Bridge | CH340, VID:PID `1A86:7523` | C |
| Line | 115200 8N1, no flow control | C |
| DTR / RTS | Must be low **before** `open()`; asserting them resets the ESP32 (auto-reset circuit). Opening with both low (pyserial, Windows) caused no reset in 3 opens | C, H |
| DTR / RTS on POSIX | Kernel raises both on open before a program can lower them; both high together should not trip the standard two-transistor auto-reset, but untested on this hub (a boot banner afterwards would show it does) | I |
| Settle | 0.3 s after open before the first command | C |
| Exclusivity | One process per port: vendor app, NINA driver, scripts are mutually exclusive | C |
| Unsolicited output | None, except a boot banner containing `SV241Pro` / `V1.3` after reset; seeing it means the hub rebooted | C, F |
| Exchange | Half-duplex: one 6-byte request, one fixed-length reply; flush input before each request | C |
| Reply timeout | 0.3–0.4 s used; replies normally arrive well within 50 ms | C |

## Frame format and checksum

Requests and replies share one shape: `$`, total length, command, payload, checksum [F].

| Offset | Field | Request | Reply |
| --- | --- | --- | --- |
| 0 | Start | `0x24` | `0x24` |
| 1 | Total length | `0x06` | payload + 4 (6, 8 or 14) |
| 2 | Command | see below | echoes request command |
| 3..n−2 | Payload | idx, value | per command |
| n−1 | Checksum | `sum(b[0..n−2]) % 255` | same |

The parser reads up to 256 buffered bytes, scans for `0x24`, takes `b[1]` as length, and requires
the checksum to equal the true remainder 0–254 [F]. A frame whose bytes sum to exactly 255 must
carry `0x00`; the vendor app's `0xFF` there fails the check and the frame is **silently dropped**
(this is what `svbony-probe.py` tests). Bad frames get no NAK; several frames in one read are all
processed [F].

Example, read state: `24 06 08 00 00 32` (36 + 6 + 8 = 50 = `0x32`).

## Command set

The firmware dispatches 11 commands: `0x00`–`0x08`, `0xFE`, `0xFF`. Others are ignored [F].

| Cmd | Name | Request `b3, b4` | Reply payload (after `24 len cmd`) | Tag |
| --- | --- | --- | --- | --- |
| `00` | Get one channel | idx, – | idx, value (outputs `00`/`FF`; idx 7–9 raw 0–255) | F |
| `01` | Set one channel | idx, value | idx, `FF` accepted / `00` refused | F |
| `02` | INA219 power | – | 4-byte sensor value: mW (uncalibrated) | F |
| `03` | Input voltage | – | sensor value: V (bus V + shunt mV/1000) | F |
| `04` | Temperature A | – | sensor value: °C + 255 — **lens probe** (DS18B20) | F, H |
| `05` | Temperature B | – | sensor value: °C + 255 — **ambient** (SHT4x) | F, H |
| `06` | Humidity | – | sensor value: %RH + 255 | F |
| `07` | INA219 current | – | sensor value: mA (uncalibrated) | F |
| `08` | Read state | – | 7 output bytes (0/1), ADJ, PWM1, PWM2 (raw 0–255) | F, H |
| `FE` | All outputs on | `FE`, – | `FE FF` (reply cmd `FE`): `24 06 FE FE FF 28` | F, H |
| `FF` | All outputs off | `FF`, – | `FF FF` (reply cmd `FF`): `24 06 FF FF FF 2A` | F, H |

**Outputs use `0x00`/`0xFF`, not 0/1.** For idx 0–6, `01` accepts only `00` (off) or `FF` (on);
anything else is refused with ack `idx, 00` and changes nothing [F]. `08` reports outputs as 0/1.
Output switching (`01` on idx 0–6, `FE`, `FF`) is confirmed on hardware [H]: acks `FF`, the `08`
read-back follows, and the physical output changes (idx 4 = DC5, idx 5 = USB-C and idx 6 = USB-A
seen on the box; DC1–DC4 had no visible load).
DC1 on: `24 06 01 00 FF 2B`; DC1 off: `24 06 01 00 00 2B`.

**All on / off.** `FE`/`FF` start an RTOS task (`TASK_SET_ALL_FORCE_ON`/`_OFF`) that switches all 7
outputs; it acts only if `b3` repeats the command byte: `24 06 FE FE 00 28`, `24 06 FF FF 00 2A`.
PWM and ADJ untouched [F]. Both act at once on hardware: the read-back 150 ms later is all off / all on [H].

**State reply** (`08`, 14 bytes):

| Byte | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Field | DC1 | DC2 | DC3 | DC4 | DC5 | USB-C | USB-A | ADJ | PWM1 | PWM2 | chk |

## Channels and scaling

| Idx | Output | GPIO | Set value (`01`) | Tag |
| --- | --- | --- | --- | --- |
| 0–4 | DC1–DC5 | 13, 12, 14, 27, 26 | `00` off / `FF` on (idx 4 = DC5 seen on the box; 0–3 read-back only) | F, H |
| 5 | USB-C | 19 | `00` / `FF` (USB VBUS: devices drop and re-enumerate) | F, H |
| 6 | USB-A | 18 | `00` / `FF` (same) | F, H |
| 7 | ADJ OUT | 25 (LEDC ch1, 40 kHz) | raw 0–255 → duty round(raw/255 × 1023) | F |
| 8 | PWM1 (dew cutoff) | 33 (LEDC ch2, 10 Hz) | same; refused during cutoff | F, H |
| 9 | PWM2 | 32 (LEDC ch3, 10 Hz) | same | F, H |

Read-back converts duty with round(duty/1023 × 255), so a write reads back unchanged [F]. Firmware
accepts the full 0–255 range; the vendor app's 253 = 100 % and "15.3 V at 253" are app
conventions [A]. ADJ OUT is a 40 kHz PWM into a regulator, so volts per raw step depend on board
hardware and cannot be read from firmware [F, I]; the 15.3 V @ 253 linear scale `sv241.py` uses
is **untested on hardware**.

NVS namespace `my-storage`: `gpio_1`…`gpio_6`, `gpio_sw6`, `pwm_a/b/c_duty` [F]. Boot defaults
before NVS: all outputs on, ADJ duty 0, PWM1 duty 1000/1023 (raw 249, the vendor's "97 %"), PWM2 0 [F].

### Sensor value encoding

All sensor replies carry 4 bytes built from x = trunc(value × 100), where value already includes
+255 for temperatures and humidity. Each byte is a quotient mod **255**, not a base-256 digit [F]:

| Byte | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- |
| Content | trunc(x/16777215) % 255 | trunc(x/65535) % 255 | trunc(x/255) % 255 | x % 255 |

For every real reading x < 65025, so b3–b4 are 0 and the exact decode is [F, H]:

```
x = 255·b5 + b6        value = x/100   (−255 for cmds 04/05/06)
```

Power (`02`, mW × 100) routinely exceeds 65025. Use
x = 65535·b4 + 255·((b5 − 2·b4) mod 255) + b6; exact except where x mod 65535 ≥ 65025 (~0.8 % of
values), where the encoding itself is ambiguous [F, brute-force checked].

A big-endian int32 decode (256·b5 + b6, as the original hold script did) reads high by b5/100:
about +1.07 °C at 20 °C, +1.19 %RH at 50 %RH, +0.04 V at 12 V. The original script subtracted
empirical offsets (255.5 for temperatures, 254 for RH) instead of 255, so its net error was about
+0.6 °C and +2.2 %RH.

| Cmd | Firmware source | Unit after decode | Notes |
| --- | --- | --- | --- |
| `02` | INA219 `getPower_mW` | mW, library default calibration | live `00 03 FD 0A` → 2596; ≈1/12.5 of vendor W, scale open |
| `03` | bus V + shunt mV/1000 | V | |
| `04` | temperature field 0 (+255) | °C | −127 = probe missing |
| `05` | temperature field 1 (+255) | °C | SHT4x − 4.0 °C applied in firmware |
| `06` | humidity (+255) | %RH | |
| `07` | INA219 `getCurrent_mA` | mA, library default calibration | |

INA219 absent: firmware stores −1 for `02`/`03`/`07`, which arrives as `00 00 00 9C` and naively
decodes as +1.56 [F, computed].

Vendor app's ×15.57 current correction implies the board shunt differs from the library default
[I]. Vendor-app power (W), v = volts, c = mA from `07` [A]:

```
P = (15.57·v·c + 269.39) / 1000
```

## Firmware behaviour

**PWM1 dew cutoff** [F, H]. Routine at `0x400d19ac` compares the lens probe (`04`) against a limit
from SHT4x ambient temperature and humidity, every 5 s:

| Ambient | Lens limit |
| --- | --- |
| > 15 °C | dew point + 7.5 °C |
| 0–15 °C | dew point + 10 °C |
| < 0 °C | 5 °C fixed |

- Lens > limit: firmware saves PWM1 duty and mode, sets duty 0, mode 2, drives LEDC ch2 to 0.
  `01` writes to idx 8 are acked `08 00` (refused) while mode = 2. ADJ and PWM2 writes are never
  refused.
- Lens ≤ limit: firmware restores the saved duty and mode itself [H]: a level written (and
  refused) during the cutoff is not what comes back, e.g. cutoff entered at raw 177, a write of
  190 refused, release returned 177. A state read during the cutoff shows PWM1 = 0, so a client
  cannot see the configured level until release.
- A `01` write to idx 8 sets mode 1.
- No hysteresis, so it works as a bang-bang thermostat at the limit [H]. At 100 % the lens sat
  0.2–0.6 °C under the limit and cycled (2026-11-20). At 70, 75 and 80 % (2026-11-22, ambient
  16.7 °C, RH 79–81 %, limit ≈ 20.6–21.1 °C) it also cycled: 53–59 % of samples in cutoff, roughly
  75 s on / 95 s off, lens within ±0.5 °C of the limit. A lower duty in that range does not
  avoid it.
- No lens probe (−127 °C), or RH ≤ 0 (dew point taken as −100 °C): cutoff cannot trip.
- Dew point: Magnus, a = 17.27, b = 237.7 [C]:

```
γ = ln(RH/100) + a·T/(b + T)        Td = b·γ/(a − γ)
```

**Timing** [C]. Sensors sampled once per 5 s; faster polling returns repeated values. Heaters
switch at 10 Hz and each INA219 read is a sub-ms snapshot, so current/power readings catch heaters
fully on or off; average them.

**Persistence** [V, F]. Settings auto-save to NVS every 5 s (V1.2+); a change followed by immediate
power loss may be lost. Reloaded from flash on boot. Heater default on first start 97 % [V].

**Ambient −4 °C fix** [F]. The V1.2 "actual = reading − 4 °C" correction is applied in firmware to
the SHT4x ambient only (`05` and the cutoff limit). Lens (`04`) is uncorrected. Clients must not
subtract 4 again.

## Open items

- [ ] See which physical jack idx 0–3 drive (DC1–DC4; idx 4 = DC5 confirmed).
- [ ] Measure ADJ OUT volts vs raw (not traceable in code: firmware only sets a 40 kHz duty on GPIO25).
- [ ] Confirm the POSIX open (DTR+RTS raised together, then dropped) does not reset the ESP32
      (hub only on Windows so far; the Windows/pyserial open is confirmed).
- [ ] Scale `02` power (1/12.5 observed vs 1/15.57 predicted by the current factor).
- [x] `04` = lens, `05` = ambient (hardware).
- [x] Output switching `01` idx 0–6, `FE`, `FF`: acks, read-back, USB-C/USB-A VBUS (hardware).
- [x] PWM1 cutoff: refusal during cutoff, saved-level restore on release (hardware).
- [x] Base-255 decode, PWM1 cutoff formula, PWM2 write/read-back (hardware).
- [x] −4 °C correction and cutoff routine traced (ambient only, `0x400d19ac`).
