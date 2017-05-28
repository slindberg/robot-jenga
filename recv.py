#recv.py
#2017 - 05 - 24
#Ernie Bodle
#I don't know how to use python but let's goooo!!!

import serial
import sys

#subprocess.Popen("sudo chmod 777 /dev/ttyUSB0")
print "sudo chmod 777 /dev/ttyUSB0", sys.argv[0]

port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=3.0)

while True:
    #port.write("123")
    print(port.read(10))
    print("\n")



