import matplotlib.pyplot as plt
import numpy
from geometry import *

def plot_path(path_fn, n_points):
    points = []
    for t in numpy.linspace(0, 1, n_points):
        points.append(path_fn(t))

    plot_points(points)
    plt.show()

def plot_joint_positions(left_arm, right_arm, path_fn, t_range):
    j_fn = lambda arm, p: two_circle_intersection(arm.p_base, p, arm.r_input, arm.r_follower)

    path_points = [ path_fn(t) for t in t_range ]
    left_points = [ j_fn(left_arm, p) for p in path_points ]
    right_points = [ j_fn(right_arm, p) for p in path_points ]

    plot_points(path_points)
    plot_points(left_points)
    plot_points(right_points)
    plt.show()

def plot_steps(step_fn, n_points, pos_curve):
    t_range = numpy.linspace(0, 1, n_points)
    steps = numpy.array([ step_fn(t) for t in pos_curve ]).transpose()

    plt.plot(t_range, steps[0,:], linestyle='', marker='.')
    plt.plot(t_range, steps[1,:], linestyle='', marker='.')
    plt.show()

def plot_points(points):
    X = [ p.x for p in points ]
    Y = [ p.y for p in points ]
    plt.plot(X, Y, linestyle='', marker='.')
    plt.axis('equal')
