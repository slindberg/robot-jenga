import math
import numpy
from scipy import integrate
from geometry import *
from paths import *

class Arm:
    def __init__(self, p_base, r_input, r_follower, gear_ratio, microsteps, steps_per_rev):
        self.p_base = p_base
        self.r_input = r_input
        self.r_follower = r_follower
        self.gear_ratio = gear_ratio
        self.microsteps = microsteps
        self.steps_per_rev = steps_per_rev

class Robot:
    def __init__(self, timer_delay, interval_size, average_arm_speed, arm_start_position, arm_left, arm_right):
        self.z_position = 0
        self.ef_position = arm_start_position
        self.ef_angle = 0

        self.timer_delay = timer_delay
        self.interval_size = interval_size
        self.average_arm_speed = average_arm_speed
        self.arm_start_position = arm_start_position
        self.arm_left = arm_left
        self.arm_right = arm_right

        easing_fn = lambda t: 3*t**2 - 2*t**3
        self.vel_fn = velocity_curve_fn(.25, .25, .1, easing_fn)

    def execute_arm_path(self, delta):
        intervals = self.calculate_arm_path(delta)
        self.ef_position += delta

        return intervals

    def execute_predefined_arm_path(self, path_name):
        if path_name == "reset":
            self.ef_position = self.arm_start_position
        else:
            path = predefined_paths[path_name]['path']
            self.ef_position = path.position_for_t(1, self.ef_position);

    def calculate_arm_path(self, delta):
        duration = delta.length()/self.average_arm_speed
        path = LinePath(delta)
        path_fn = path.path_fn(self.ef_position)

        try:
            return self.step_intervals_for_path(path_fn, duration)
        except ValueError:
            raise Error('Invalid path geometry')

    def samples_for_duration(self, duration):
        return int(math.ceil(duration/(self.interval_size*self.timer_delay)))

    def step_intervals_for_path(self, path_fn, duration):
        n_points = self.samples_for_duration(duration)
        step_fn = self.step_fn(path_fn)

        return self.step_intervals_for_steps(step_fn, n_points)

    def step_intervals_for_steps(self, step_fn, n_points):
        pos_curve = self.pos_curve_for_n(n_points)
        last_steps = [ 0, 0 ]
        step_intervals = numpy.zeros((n_points, 2))

        for i in range(n_points):
            steps_i = step_fn(pos_curve[i])
            delta_steps = numpy.subtract(steps_i, last_steps)
            step_intervals[i] = delta_steps
            last_steps = steps_i

        # Remove any intervals where neither arm moves
        step_intervals = step_intervals[~numpy.all(step_intervals == 0, axis=1)]

        return step_intervals

    def pos_curve_for_n(self, n_points):
        t_range = numpy.linspace(0, 1, n_points)
        pos_for_t = lambda t: integrate.quad(self.vel_fn, 0, t)
        scale, _ = pos_for_t(1);

        # TODO: don't keep recalculating this integral, just do it manually
        return [ pos_for_t(t)[0]/scale for t in t_range ]

    def step_fn(self, path_fn):
        p_initial = path_fn(0)
        left_step_fn = self.step_fn_for_p(self.arm_left, p_initial)
        right_step_fn = self.step_fn_for_p(self.arm_right, p_initial)
        step_fn_both = lambda p: [ left_step_fn(p), right_step_fn(p) ]

        return lambda t: step_fn_both(path_fn(t))

    def step_fn_for_p(self, arm, p_initial):
        theta_0 = input_angle(arm.p_base, p_initial, arm.r_input, arm.r_follower)
        steps_per_rad = arm.gear_ratio*arm.steps_per_rev*arm.microsteps/(2*math.pi)

        return lambda p: round((input_angle(arm.p_base, p, arm.r_input, arm.r_follower) - theta_0)*steps_per_rad)

    def output_step_intervals(self, path_name):
        path_data = predefined_paths['enter']
        path_fn = path_data['path'].path_fn(path_data['start'])
        step_intervals = self.step_intervals_for_path(path_fn, path_data['duration'])

        numpy.savetxt(path_name + "_step_intervals.csv", step_intervals.transpose(), delimiter=",", fmt='%d')

