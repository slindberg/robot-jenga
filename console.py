#!/usr/bin/env python

import sys
import platform
import serial

#subprocess.Popen("sudo chmod 777 ")
# print "sudo chmod 777 /dev/ttyUSB0", sys.argv[0]

os = platform.system()

devices = {
    'Darwin': '/dev/tty.SLAB_USBtoUART',
    'Linux': '/dev/ttyUSB0',
}

port = serial.Serial(devices[os], baudrate=9600, timeout=3.0)

# port.write("123")
# print(port.read(10))

while 1:
    try:
        command = sys.stdin.readline()
    except KeyboardInterrupt:
        break

    if not command:
        break

    command = command.rstrip()
    
    port.write(command)
    response = port.read(4)
    print(response)


