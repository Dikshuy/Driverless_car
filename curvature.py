import numpy as np


def curvature_fn(x, y):
    k = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for j in range(0, len(x)):
        k[j, 0] = (dx[j, 0] * ddy[j, 0] - dy[j, 0] * ddx[j, 0]) / (dx[j, 0] * dx[j, 0] + dy[j, 0] * dy[j, 0])

    return k
