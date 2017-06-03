#!/usr/bin/env python

import sys
from controller import Controller, NotConnectedError
from commands import CommandError
from robot_instance import robot

controller = Controller(robot)

try:
    controller.connect()
except Exception as err:
    print(err.message)

while True:
    try:
        line = sys.stdin.readline().rstrip()

        if not line:
            continue

        args = line.split()
        command = args.pop(0)

        response = controller.execute_command(command, args)

        # Response is a generator, and we want to print each char
        # immediately after it is received
        for c in response:
            sys.stdout.write(c)
            sys.stdout.flush()

        print("")
    except (NotConnectedError, CommandError) as err:
        print('Error: ' + err.message)
    except KeyboardInterrupt:
        sys.exit();
