from path_planning import *

arm_left = Arm(
    p_base = Point(90, 104.45625),
    r_input = 336.675,
    r_follower = 232.0,
    gear_ratio = 96/20,
    microsteps = 16,
    steps_per_rev = 200,
)

arm_right = Arm(
    p_base = Point(30, 244.45625),
    r_input = 243.325,
    r_follower = 190.0,
    gear_ratio = 72/20,
    microsteps = 16,
    steps_per_rev = 200,
)

timer_delay = 1/(16e6/(64*64))
interval_size = 100
n_fn = samples_for_duration(timer_delay, interval_size)

easing_fn = lambda t: 3*t**2 - 2*t**3
vel_curve = velocity_curve_for_t(.25, .25, .1, easing_fn)

# Enter path
n_points = n_fn(3)
path_fn = enter_path_for_t()
step_fn = steps_for_t_both(arm_left, arm_right, path_fn)

step_intervals = generate_intervals(step_fn, vel_curve, n_points)
numpy.savetxt("enter_intervals.csv", step_intervals.transpose(), delimiter=",", fmt='%d')


