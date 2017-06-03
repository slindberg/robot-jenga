#!/usr/bin/env python

import sys
import platform
import time
import serial
from commands import *

os = platform.system()

devices = {
    'Darwin': '/dev/tty.SLAB_USBtoUART',
    'Linux': '/dev/ttyUSB0',
}

# To write to the serial port, we have to have write permission on the device
if os == 'Linux':
    print "sudo chmod 777 /dev/ttyUSB0", sys.argv[0]

port = serial.Serial(devices[os], baudrate=9600, timeout=None)

class ResponseError(Exception):
    pass

class NoCommandError(Exception):
    pass

def read_command():
    line = sys.stdin.readline().rstrip()

    if not line:
        raise NoCommandError()

    parts = line.split()
    command = parts.pop(0)

    return (command, parts)

def send(data):
    port.write(data)

def recieve():
    length_hex = port.read(2)

    if length_hex:
        try:
            length = ord(length_hex.decode('hex'))
        except:
            raise ResponseError("Bad response length: '" + length_hex + "'")

        return port.read(length)

while True:
    print('Waiting on turn signal');
    is_turn = False
    while not is_turn:
        send(turn_check_command())
        is_turn = recieve() == '1'
        time.sleep(0.05)

    print('Beginning turn');
    send(begin_turn_command())
    print(recieve())

    while True:
        try:
            (command, args) = read_command()
            send(handle_command(command, args))
            response = recieve()
            print(response)

            # This is the turn ended signal
            if response == '!':
                print('Ending turn');
                while is_turn:
                    send(turn_check_command())
                    is_turn = recieve() == '1'
                    time.sleep(0.05)
                break
        except KeyboardInterrupt:
            sys.exit();
        except (CommandError, ResponseError) as err:
            print('Error: ' + err.message)
            continue
        except NoCommandError:
            continue

        # Debugging: just continuously read and print each byte
        # while True:
        #     response = port.read()
        #     sys.stdout.write(response)
        #     sys.stdout.flush()

