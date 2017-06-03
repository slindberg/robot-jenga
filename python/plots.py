import matplotlib.pyplot as plt
import numpy

def plot_path(path_fn, n_points):
    points = []
    for t in numpy.linspace(0, 1, n_points):
        points.append(path_fn(t))

    X = [ p.x for p in points ]
    Y = [ p.y for p in points ]
    plt.plot(X, Y)
    plt.axis('equal')
    plt.show()

