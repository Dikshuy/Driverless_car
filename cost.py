import numpy as np
import math


dyn1_x = []
dyn1_y = []


def curvature_fn(x, y):
    k_matrix = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for j in range(0, len(x)):
        k_matrix[j, 0] = (dx[j, 0] * ddy[j, 0] - dy[j, 0] * ddx[j, 0]) / (dx[j, 0] * dx[j, 0] + dy[j, 0] * dy[j, 0])

    return k_matrix


def cost(xd, yd, x_cen, y_cen, phase):
    if max(curvature_fn(xd, yd)) <= 0.15:
        if phase == 2:
            cost1 = 0
            cost2 = np.sum(math.sqrt(np.square(x_cen - xd) + np.square(y_cen - yd)))
            total_cost = 2*cost1 + 0.5*cost2
        else:
            cost1 = 1 / max(min(math.sqrt(np.square(xd- dyn1_x) + np.square(yd - dyn1_y)))-2, 0.01)
            cost2 = np.sum(math.sqrt(np.square(x_cen - xd) + np.square(y_cen - yd)))
            total_cost = 2 * cost1 + 0.5 * cost2

    else:
        total_cost = math.inf

    return total_cost



