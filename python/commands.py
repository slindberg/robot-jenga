from geometry import Point
from controller import robot

# (1 rev / 4mm)*(1000 steps / 1 rev)*(14.5mm / block) = 3625 steps/block
block_height = 3625

class CommandError(Exception):
    pass

def handle_command(command, args):
    if command == 'z':
        return zaxis_command(*args)
    elif command == 'arm':
        return arm_command(*args)
    elif command == 'ef':
        return ef_command(*args)
    elif command == 'fire':
        return fire_command()
    elif command == 'end':
        return end_turn_command()
    else:
        raise CommandError("Unknown command'" + command + "'")

predefined_paths = {
    'enter': 1,
    'exit': 2,
    'reset': 3,
}

def zaxis_command(direction, distance = '0'):
    if direction == 'home':
        return 'H'
    if direction == 'up':
        sign = 1
    elif direction == 'down':
        sign = -1
    else:
        raise CommandError("Unknown direction'" + direction + "'")

    if distance == 'block':
        distance = block_height
    elif distance == 'story':
        distance = 2*block_height
    else:
        try:
           distance = int(distance)
        except ValueError:
            raise CommandError("Distance must be integer, got '" + distance + "'")


    return 'Z' + tohex(sign*distance, 16)

def arm_command(path_name, distance = '0'):
    if path_name in predefined_paths:
        robot.execute_predefined_arm_path(path_name)
        path_number = predefined_paths[path_name]
        return 'P' + tohex(path_number, 8)

    try:
       distance = int(distance)
    except ValueError:
        raise CommandError("Distance must be integer, got '" + distance + "'")

    if path_name == 'east':
        delta = Point(distance, 0)
    elif path_name == 'west':
        delta = Point(-distance, 0)
    elif path_name == 'north':
        delta = Point(0, distance)
    elif path_name == 'south':
        delta = Point(0, -distance)
    else:
        raise CommandError("Unknown arm path '" + path_name + "'")

    if distance == 0:
        raise CommandError('Must specify nonzero distance')

    intervals = robot.execute_arm_path(delta)
    path_length = intervals.shape[0]

    left_bytes = ''.join([ tohex(i, 8) for i in intervals[:,0] ])
    right_bytes = ''.join([ tohex(i, 8) for i in intervals[:,1] ])

    encoded_path = tohex(path_length, 8) + left_bytes + right_bytes

    return 'A' + encoded_path

def ef_command(direction, distance):
    if direction == 'ccw':
        sign = 1
    elif direction == 'cw':
        sign = -1
    else:
        raise CommandError("Unknown direction'" + direction + "'")

    try:
       distance = int(distance)
    except ValueError:
        raise CommandError("Distance must be integer, got '" + distance + "'")

    return 'R' + tohex(sign*distance, 16)

def fire_command():
    return 'F'

def turn_check_command():
    return 'T'

def begin_turn_command():
    return 'B'

def end_turn_command():
    return 'E'

def tohex(value, bit_width):
    hex_str = hex((value + (1 << bit_width)) % (1 << bit_width))
    return hex_str.rstrip("L").lstrip("0x").zfill(bit_width / 4)
