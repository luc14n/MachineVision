import serial
import time
port = "/dev/cu.usbmodem3175345F31302"
s = serial.Serial(port, 115200, timeout=1)
s.write(b'\x03\x03\x03')
time.sleep(0.5)
s.close()
