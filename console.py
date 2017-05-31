#!/usr/bin/env python

import sys
import platform
import serial

os = platform.system()

devices = {
    'Darwin': '/dev/tty.SLAB_USBtoUART',
    'Linux': '/dev/ttyUSB0',
}

# To write to the serial port, we have to have write permission on the device
if os == 'Linux':
    print "sudo chmod 777 /dev/ttyUSB0", sys.argv[0]

port = serial.Serial(devices[os], baudrate=9600, timeout=3.0)

while True:
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

    # Debugging: just continuously read and print each byte
    # while True:
    #     response = port.read()
    #     sys.stdout.write(response)
    #     sys.stdout.flush()
