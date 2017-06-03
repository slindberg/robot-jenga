#!/usr/bin/env python

from controller import *
from robot_instance import robot

controller = Controller(robot)

try:
    controller.play()
except NotConnectedError as err:
    print(err.message)
