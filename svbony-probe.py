"""
svbony-probe.py: one-off hardware probe testing the SV241 Pro checksum rule (Windows, COM6).

Question it answers: when a request frame's bytes sum to exactly 255, which checksum does the
firmware accept? The vendor app sends 0xFF there; the firmware compares against the true
remainder sum % 255 = 0x00 (see docs/PROTOCOL.md, "Frame format and checksum").

Steps (PWM1 = set index 8, raw 204 = 0xCC; 24+06+01+08+CC = 0xFF = 255):
  1. Read state (cmd 08) and remember PWM1 (state reply byte 11).
  2. Send 24 06 01 08 CC FF (app-style checksum): expect no ack and PWM1 unchanged (dropped).
  3. Send 24 06 01 08 CC 00 (firmware-style): expect ack 24 06 01 08 FF .. and PWM1 = 204.
  4. Write the original PWM1 back with a correctly computed checksum.

Note: if the dew cutoff is active, step 3 is refused (ack byte 00) and PWM1 stays 0.
Requires pyserial. DTR/RTS are set low before open() so the ESP32 is not reset. Edit "COM6"
for another port. Superseded for everyday use by sv241.py (`sv241.py raw ...`).
"""
import serial, time
# Open with DTR/RTS low (set before open), then let the port settle 0.3 s.
p = serial.Serial(); p.port="COM6"; p.baudrate=115200; p.timeout=0.3; p.dtr=False; p.rts=False; p.open(); time.sleep(0.3)
# st(): flush input, send "read state" (24 06 08 00 00 32), return the 14-byte reply.
st = lambda: (p.reset_input_buffer(), p.write(bytes([0x24,6,8,0,0,0x32])), time.sleep(0.05), p.read(14))[-1]
orig = st()[11]   # byte 11 of the state reply = PWM1 raw
p.write(bytes([0x24,6,1,8,204,0xFF])); time.sleep(0.15); print("app-style  ack:", p.read(6).hex(" ") or "NONE", " PWM1 now", st()[11])
p.write(bytes([0x24,6,1,8,204,0x00])); time.sleep(0.15); print("fw-style   ack:", p.read(6).hex(" ") or "NONE", " PWM1 now", st()[11])
# Restore: 43 = 0x24 + 0x06 + 0x01 (header + cmd), + idx 8, + value; checksum = sum % 255.
s = 43+8+orig; p.write(bytes([0x24,6,1,8,orig,s%255])); time.sleep(0.15); p.read(6); print("restored", st()[11])
