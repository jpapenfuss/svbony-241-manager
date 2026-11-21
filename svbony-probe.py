import serial, time
p = serial.Serial(); p.port="COM6"; p.baudrate=115200; p.timeout=0.3; p.dtr=False; p.rts=False; p.open(); time.sleep(0.3)
st = lambda: (p.reset_input_buffer(), p.write(bytes([0x24,6,8,0,0,0x32])), time.sleep(0.05), p.read(14))[-1]
orig = st()[11]
p.write(bytes([0x24,6,1,8,204,0xFF])); time.sleep(0.15); print("app-style  ack:", p.read(6).hex(" ") or "NONE", " PWM1 now", st()[11])
p.write(bytes([0x24,6,1,8,204,0x00])); time.sleep(0.15); print("fw-style   ack:", p.read(6).hex(" ") or "NONE", " PWM1 now", st()[11])
s = 43+8+orig; p.write(bytes([0x24,6,1,8,orig,s%255])); time.sleep(0.15); p.read(6); print("restored", st()[11])