import numpy as np
import math


dyn1_x = []
dyn1_y = []
total_cost = 0


def curvature_fn(x, y):
    k_matrix = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for j in range(0, len(x)):
        k_matrix[j, 0] = (dx[j, 0] * ddy[j, 0] - dy[j, 0] * ddx[j, 0]) / (dx[j, 0] * dx[j, 0] + dy[j, 0] * dy[j, 0])

    return k_matrix


def cost(xd1, yd1, x_centre, y_centre):
    global total_cost
    menger_curvature = np.zeros(11)
    menger_curvature[:] = curvature_fn(xd1, yd1)
    if max(menger_curvature[:]) <= 0.15:
        # if phase_chosen == 2:
        #     cost1 = 0
        #     cost2 = np.sum(np.sqrt(np.add(np.square(x_centre - xd1), np.square(y_centre - yd1))))
        #     total_cost = 2 * cost1 + 0.5 * cost2
        #     print(total_cost)
        # else:
        cost1 = 1 / (max(min(np.sqrt(np.add(np.square(xd1 - dyn1_x), np.square(yd1 - dyn1_y)))) - 2, 0.01))
        cost2 = np.sum(np.sqrt(np.add(np.square(x_centre - xd1), np.square(y_centre - yd1))))
        total_cost = 2 * cost1 + 0.5 * cost2

    else:
        total_cost = math.inf

    return total_cost
