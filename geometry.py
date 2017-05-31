import math

class Point:
    def __init__(self, x_init, y_init):
        self.x = x_init
        self.y = y_init

    def __repr__(self):
        return "".join([ "Point(", str(self.x), ",", str(self.y), ")" ])

    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)

    def __sub__(self, p):
        return Point(self.x - p.x, self.y - p.y)

    def __mul__(self, s):
        return Point(self.x*s, self.y*s)

    def __div__(self, s):
        return Point(self.x/s, self.y/s)

    def __neg__(self):
        return Point(-self.x, -self.y)

    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def atan2(self):
        return math.atan2(self.y, self.x)

    def atan2_2pi(self):
        theta = self.atan2()

        # Convert -pi .. pi to 0 .. 2pi
        return theta*(theta >= 0) + (theta + 2*math.pi)*(theta < 0)

    def rotate(self, theta):
        return Point(self.x*math.cos(theta), self.y*math.sin(theta))

# Utility methods

def two_circle_intersection(p0, p1, r0, r1, ccw = True):
    # distance between the two circle center points
    d = (p1 - p0).length()

    # distance from center of first circle to the centerline between two
    # circular intersection points
    a = (r0**2 - r1**2 + d**2)/(2*d)

    # normal distance from line between the points and the intersections
    h = math.sqrt(r0**2 - a**2)

    # intersection of the line between the points and the line between the
    # two circle intersection points
    p2 = p0 + (p1 - p0)*(a/d)

    # offset distance from the p2
    offset = Point(p1.y - p0.y, p0.x - p1.x)*h/d;

    # find one of the intersections using p_2 and h
    return p2 - (offset if ccw else -offset)

def circle_center(p, r, ccw):
    # simplified version of two circle intersection
    d = p.length()
    h = math.sqrt(r**2 - d**2/4)
    offset = Point(p.y, -p.x)*h/d;

    return p/2 - (offset if ccw else -offset)

def input_angle(p0, p1, r0, r1):
    p_joint = two_circle_intersection(p0, p1, r0, r1)
    return (p_joint - p0).atan2_2pi()

def velocity_curve_fn(t_a, t_d, min_v, easing):
    return lambda t: s_curve_for_t(t, t_a, t_d, min_v, easing)

def s_curve_for_t(t, t_a, t_d, min_x, easing_fn):
    if t <= t_a:
        # acceleration
        x = easing_fn(t/t_a)
    elif t >= 1 - t_d:
        # deceleration
        x = 1 - easing_fn((t - (1 - t_d))/t_d)
    else:
        # steady state
        x = easing_fn(1)

    return x*(1 - min_x) + min_x
