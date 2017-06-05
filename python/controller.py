import sys
import platform
import time
import serial
from commands import *

start_char = ':'
end_char = ';'
reconnect_timeout = 10 # seconds
receive_timeout = 5 # seconds
write_timeout = 5 # seconds

class NotConnectedError(Exception):
    pass

class Controller:
    def __init__(self, robot):
        self.commander = CommandProtocol(robot)
        self.port = None

    def connect(self):
        os = platform.system()

        devices = {
            'Darwin': '/dev/tty.SLAB_USBtoUART',
            'Linux': '/dev/ttyUSB0',
        }

        # To write to the serial port, we have to have write permission on the device
        if os == 'Linux':
            print 'sudo chmod 777 /dev/ttyUSB0', sys.argv[0]

        try:
            self.port = serial.Serial(devices[os], baudrate=9600, timeout=receive_timeout, write_timeout=write_timeout)
        except serial.SerialException as err:
            raise NotConnectedError(str(err))

    def send(self, data):
        if not self.port:
            raise NotConnectedError("Not connected, cannot send data")

        data_received = False

        # Attempt to send until the response start character is received
        while not data_received:
            try:
                self.port.write(data)
                data_received = self.port.read() == start_char
            except serial.SerialTimeoutException:
                print('Receive timeout reached')
                return []

        return self.receive()

    def receive(self):
        if not self.port:
            raise NotConnectedError('Not connected, cannot receive data')

        last_char_ts = time.clock()

        # Read until the termination character is received or timeout
        while True:
            char = self.port.read()

            if not char:
                print('Receive timeout reached')
                break

            if char == end_char:
                break

            yield char

    def execute_command(self, command, args):
        data = self.commander.handle_command(command, args)
        return self.send(data) if data else []

    def wait_for_turn(self):
        print('Waiting on turn signal')

        is_turn = False
        while not is_turn:
            response = self.send(self.commander.turn_check_command())
            is_turn = response == '1'
            time.sleep(0.05)

    def play(self):
        self.connect()

        while True:
            self.wait_for_turn()
            self.play_turn()

    def play_turn(self):
        self.execute_command('begin')
        self.execute_command('arm', 'enter')
        # data = self.execute_command('scan', 'west', 15)

        # TODO: somehow wait for commands to complete

        self.execute_command('end')
