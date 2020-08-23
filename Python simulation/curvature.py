import numpy as np


def curvature_fn(x, y):
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)
    A = np.multiply(dx, ddy)
    B = np.multiply(dy, ddx)
    C = np.subtract(A, B)
    D = np.multiply(dx, dx)
    E = np.multiply(dy, dy)
    F = np.add(D, E)
    G = np.power(F, 1.5)
    k_matrix = np.divide(C, G)
    return k_matrix
