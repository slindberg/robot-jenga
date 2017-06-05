import math
from geometry import *

class Path:
    def path_fn(self, start):
        return lambda t: self.position_for_t(t, start)

    def position_for_t(self, t, start):
        raise NotImplementedError('Must implement position_for_t()')

class LinePath(Path):
    def __init__(self, delta):
        self.delta = delta
        self.length = delta.length()

    def position_for_t(self, t, start = Point(0, 0)):
        return start + self.delta*t

class ArcPath(Path):
    def __init__(self, delta, radius, ccw):
        self.radius = radius
        self.center = circle_center(delta, radius, ccw)
        self.theta0 = (-self.center).atan2()
        self.theta1 = (delta - self.center).atan2()
        self.theta_scale = self.theta1 - self.theta0
        self.length = radius*abs(self.theta_scale)

    def position_for_t(self, t, start = Point(0, 0)):
        theta = self.theta0 + t*self.theta_scale
        arc_end = Point(1, 1).rotate(theta)*self.radius
        return start + self.center + arc_end

class CompoundPath(Path):
    def __init__(self, segments):
        self.segments = segments
        self.length = sum(s.length for s in segments)

    def position_for_t(self, t, start = Point(0, 0)):
        cur_length = 0
        for s in self.segments:
            if t <= (s.length + cur_length) / self.length:
                t0 = cur_length/self.length
                t1 = (cur_length + s.length)/self.length
                return s.position_for_t((t - t0)/(t1 - t0), start)

            cur_length += s.length
            start = s.position_for_t(1, start)

class ReversePath(Path):
    def __init__(self, path):
        self.forward_path = path
        self.offset = path.position_for_t(1)

    def position_for_t(self, t, start = Point(0, 0)):
        return start - self.offset + self.forward_path.position_for_t(1-t)

enter_path = CompoundPath([
    ArcPath(Point(-221.49248,184.9257), 340.2250, False),
    LinePath(Point(-107.5,230)),
])

exit_path = ReversePath(enter_path)

side_path = CompoundPath([
    LinePath(Point(82.5,0)),
    ArcPath(Point(25,25), 25, True),
    LinePath(Point(0,82.5)),
])

predefined_paths = {
    'enter': {
        'duration': 3,
        'start': Point(-8.50752, -184.9257),
        'path': enter_path,
    },
    'exit': {
        'duration': 3,
        'start': Point(-337.5, 230),
        'path': exit_path,
    },
    'side': {
        'duration': 3,
        'start': Point(-337.5, 230),
        'path': side_path,
    }
}
