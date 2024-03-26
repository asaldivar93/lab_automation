import serial
import ast

port = serial.Serial(port="/dev/ttyUSB0", baudrate=230400, timeout=2)

port.write("M0 0,!".encode())
input = port.readline()

if port.inWaiting() > 0:
    input = port.readline()

print(input)
port.close()
