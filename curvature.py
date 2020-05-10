import numpy as np
import sympy as sym


def curvature_fn(x, y):
    k = np.zeros(len(x), 1)
    dx = sym.diff(x)
    ddx = sym.diff(dx)
    dy = sym.diff(y)
    ddy = sym.diff(dy)

    for j in range(0, len(x)):
        k[j, 0] = (dx(j, 0) * ddy(j, 0) - dy(j, 0) * ddx(j, 0)) / (dx(j, 0) * dx(j, 0) + dy(j, 0) * dy(j, 0))

    return k
