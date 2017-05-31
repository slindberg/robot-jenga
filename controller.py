from robot import *

robot = Robot(
    timer_delay = 1/(16e6/(64*64)),
    interval_size = 100,
    average_arm_speed = 20, # mm/sk
    arm_start_position = Point(-8.50752, -184.9257),
    arm_left = Arm(
        p_base = Point(90, 104.45625),
        r_input = 336.675,
        r_follower = 232.0,
        gear_ratio = 96/20,
        microsteps = 16,
        steps_per_rev = 200,
    ),
    arm_right = Arm(
        p_base = Point(30, 244.45625),
        r_input = 243.325,
        r_follower = 190.0,
        gear_ratio = 72/20,
        microsteps = 16,
        steps_per_rev = 200,
    ),
)

robot.output_step_intervals('enter');
