#!/usr/bin/env python

import sys
import platform
import serial
from commands import handle_command, CommandError

os = platform.system()

devices = {
    'Darwin': '/dev/tty.SLAB_USBtoUART',
    'Linux': '/dev/ttyUSB0',
}

# To write to the serial port, we have to have write permission on the device
if os == 'Linux':
    print "sudo chmod 777 /dev/ttyUSB0", sys.argv[0]

port = serial.Serial(devices[os], baudrate=9600, timeout=1.0)

while True:
    try:
        line = sys.stdin.readline()
    except KeyboardInterrupt:
        break

    if not line:
        continue

    parts = line.rstrip().split()
    command = parts.pop(0)

    try:
        data = handle_command(command, parts)
    except CommandError as err:
        print('Error: ' + err.message)
        continue

    port.write(data)

    # Response
    length_hex = port.read(2)

    if length_hex:
        try:
            length = ord(length_hex.decode('hex'))
        except:
            print("Bad response length: '" + length_hex + "'")
            continue

        response = port.read(length)
        print(response)

    # Debugging: just continuously read and print each byte
    # while True:
    #     response = port.read()
    #     sys.stdout.write(response)
    #     sys.stdout.flush()
