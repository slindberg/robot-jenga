import math
import numpy
from scipy import integrate

class Arm:
    def __init__(self, p_base, r_input, r_follower, gear_ratio, microsteps, steps_per_rev):
        self.p_base = p_base
        self.r_input = r_input
        self.r_follower = r_follower
        self.gear_ratio = gear_ratio
        self.microsteps = microsteps
        self.steps_per_rev = steps_per_rev

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

    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def rotate(self, theta):
        return Point(self.x*math.cos(theta), self.y*math.sin(theta))

def generate_intervals(step_fn, vel_curve, n_points):
    pos_scale, err = integrate.quad(vel_curve, 0, 1)
    t_curve = numpy.linspace(0, 1, n_points)

    last_steps = [ 0, 0 ]
    step_intervals = numpy.zeros((n_points, 2))

    # TODO: don't keep recalculating this integral, just do it manually
    for i in range(n_points):
        t, err = integrate.quad(vel_curve, 0.0, t_curve[i])
        steps_i = step_fn(t/pos_scale)
        delta_steps = numpy.subtract(steps_i, last_steps)
        step_intervals[i] = delta_steps
        last_steps = steps_i

    # Remove any intervals where neither arm moves
    step_intervals = step_intervals[~numpy.all(step_intervals == 0, axis=1)]

    return step_intervals

def steps_for_t_both(arm_left, arm_right, path_fn):
    p_initial = path_fn(0)

    step_for_p_fn = steps_for_pos_both(arm_left, arm_right, p_initial)

    return lambda t: step_for_p_fn(path_fn(t))

def steps_for_pos_both(arm_left, arm_right, p_initial):
    left_step_fn = steps_for_pos(arm_left, p_initial)
    right_step_fn = steps_for_pos(arm_right, p_initial)

    return lambda p: [ left_step_fn(p), right_step_fn(p) ]

def steps_for_pos(arm, p_initial):
    theta_0 = input_angle(arm.p_base, p_initial, arm.r_input, arm.r_follower)
    steps_per_rad = arm.gear_ratio*arm.steps_per_rev*arm.microsteps/(2*math.pi)

    return lambda p: round((input_angle(arm.p_base, p, arm.r_input, arm.r_follower) - theta_0)*steps_per_rad)

def input_angle(p_0, p_1, r_0, r_1):
    p_joint = circle_intersection(p_0, p_1, r_0, r_1)
    p_arc = p_joint - p_0
    theta = math.atan2(p_arc.y, p_arc.x)

    # Convert -pi .. pi to 0 .. 2pi
    return theta*(theta >= 0) + (theta + 2*math.pi)*(theta < 0)

def circle_intersection(p_0, p_1, r_0, r_1):
    # distance between the two circle center points
    d = (p_1 - p_0).length()

    # distance from center of first circle to the centerline between two
    # circular intersection points
    a = (r_0**2 - r_1**2 + d**2)/(2*d)

    # normal distance from line between the points and the intersections
    h = math.sqrt(r_0**2 - a**2)

    # intersection of the line between the points and the line between the
    # two circle intersection points
    p_2 = p_0 + (p_1 - p_0)*(a/d)

    # for now, just find one of the intersections using p_2 and h
    return Point(p_2.x - h*(p_1.y - p_0.y)/d, p_2.y + h*(p_1.x - p_0.x)/d)

def position_curve(vel_curve, n_points):
    p = []
    t_curve = numpy.linspace(0, 1, n_points)
    pos_scale = integrate.quad(vel_curve(t), 0, 1)

    # TODO: this integral is wasteful and easy to do manually
    for i in range(n_points):
        p.append(integrate.quad(vel_curve, 0, t_curve[i])/pos_scale)

    return p

def velocity_curve_for_t(t_a, t_d, min_v, easing):
    return lambda t: s_curve(t, t_a, t_d, min_v, easing)

def s_curve(t, t_a, t_d, min_x, easing_fn):
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

def samples_for_duration(interval_size, timer_delay):
    return lambda duration: int(math.ceil(duration/(interval_size*timer_delay)))

def enter_path_for_t():
    p_arc_start = Point(-8.50752, -184.9257)
    p_arc_end = Point(-230, 0)
    p_line_end = Point(-337.5, 230)
    p_arc_center = Point(78.2205, 144.0596)

    arc_path_fn, l_arc = arc_path_for_t(p_arc_start, p_arc_end, p_arc_center)
    line_path_fn, l_line = line_path_for_t(p_arc_end, p_line_end)

    return combine_paths_for_t(arc_path_fn, l_arc, line_path_fn, l_line)

def side_path_for_t():
    p_line_1_start = Point(-337.5, 230)
    p_arc_start = Point(-255, 230)
    p_arc_end = Point(-230, 255)
    p_line_2_end = Point(-230, 337.5)
    p_arc_center = Point(-255, 255)

    line_1_path_fn, l_line_1 = line_path_for_t(p_line_1_start, p_arc_start)
    arc_path_fn, l_arc = arc_path_for_t(p_arc_start, p_arc_end, p_arc_center)
    line_2_path_fn, l_line_2 = line_path_for_t(p_arc_end, p_line_2_end)

    path_fn_1 = combine_paths_for_t(line_1_path_fn, l_line_1, arc_path_fn, l_arc)

    return combine_paths_for_t(path_fn_1, l_line_1 + l_arc, line_2_path_fn, l_line_2)

def reverse_path_for_t(path_fn):
    return lambda t: path_fn(1 - t)

def line_path_for_t(p_start, p_end):
    path_fn = lambda t: p_start + (p_end - p_start)*t
    length = (p_start - p_end).length()
    return (path_fn, length)

def arc_path_for_t(p_start, p_end, p_center):
    theta_0 = math.atan2(p_start.y - p_center.y, p_start.x - p_center.x)
    theta_1 = math.atan2(p_end.y - p_center.y, p_end.x - p_center.x)
    theta_scale = theta_1 - theta_0
    radius = (p_start - p_center).length()
    length = radius*abs(theta_1 - theta_0)

    path_fn = lambda t: p_center + (Point(1,1)*radius).rotate(t*theta_scale + theta_0)

    return (path_fn, length)

def combine_paths_for_t(path_fn_1, l_path_1, path_fn_2, l_path_2):
    l_path = l_path_1 + l_path_2
    ratio_1 = l_path_1/l_path
    ratio_2 = l_path_2/l_path

    return lambda t: path_fn_1(t/ratio_1) if t < ratio_1 else path_fn_2((t - ratio_1)/ratio_2)
