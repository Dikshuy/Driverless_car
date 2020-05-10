import math

import matplotlib.pyplot as plt
import numpy as np


def environment_load():
    rows = 11
    cols = 300
    str_cols = 50
    first_cols = 75
    second_cols = 100
    straight_cols2 = 150
    third_cols = 250

    ogm = np.zeros((4, rows, cols))

    dx = 1.0
    dy = 1.0

    c_row = 6
    x = 0
    y = 0

    for i in range(0, str_cols):
        ogm[0, c_row - 1, i] = x
        ogm[1, c_row - 1, i] = y
        ogm[2, c_row - 1, i] = 0
        ogm[3, c_row - 1, i] = 0
        x = x + dx

    r0 = 20
    dth = 0.25 * math.pi / (first_cols - str_cols)
    x = ogm[0, c_row - 1, str_cols - 1]
    y = ogm[1, c_row - 1, str_cols - 1]
    th = dth

    for i in range(str_cols, first_cols):
        ogm[0, c_row - 1, i] = x + r0 * math.sin(th)
        ogm[1, c_row - 1, i] = y + r0 * (1 - math.cos(th))
        ogm[2, c_row - 1, i] = th
        ogm[3, c_row - 1, i] = th
        th = th + dth

    for i in range(str_cols, first_cols):
        print(x + r0 * math.sin(th))
        th = th + dth

    dth = 0.25 * math.pi / (second_cols - first_cols)
    th = 0.25 * math.pi + dth
    x = ogm[0, c_row - 1, first_cols - 1]
    y = ogm[1, c_row - 1, first_cols - 1]

    for i in range(first_cols, second_cols):
        ogm[0, c_row - 1, i] = x + r0 * math.cos(0.25 * math.pi) - r0 * math.cos(th)
        ogm[1, c_row - 1, i] = y + r0 * math.sin(th) - r0 * math.sin(0.25 * math.pi)
        ogm[2, c_row - 1, i] = 1.5 * math.pi - th
        ogm[3, c_row - 1, i] = 0.5 * math.pi - th
        th = th + dth

    x = ogm[0, c_row - 1, second_cols - 1]
    y = ogm[1, c_row - 1, second_cols - 1]
    for i in range(second_cols + 1, straight_cols2):
        ogm[0, c_row - 1, i] = x + dx
        ogm[1, c_row - 1, i] = y
        ogm[2, c_row - 1, i] = math.pi
        ogm[3, c_row - 1, i] = 0
        x = x + dx

    dth = math.pi / (third_cols - straight_cols2)
    th = math.pi / 2 + dth
    x = ogm[0, c_row - 1, straight_cols2 - 1]
    y = ogm[1, c_row - 1, straight_cols2 - 1]
    for i in range(straight_cols2 + 1, third_cols):
        ogm[0, c_row - 1, i] = x - r0 * math.cos(th)
        ogm[1, c_row - 1, i] = y - r0 * (1 - math.sin(th))
        ogm[2, c_row - 1, i] = th
        ogm[3, c_row - 1, i] = 0.5 * math.pi - th
        th = th + dth

    x = ogm[0, c_row - 1, third_cols - 1]
    y = ogm[1, c_row - 1, third_cols - 1]
    for i in range(third_cols + 1, cols):
        ogm[0, c_row - 1, i] = x - dx
        ogm[1, c_row - 1, i] = y
        ogm[2, c_row - 1, i] = 0
        ogm[3, c_row - 1, i] = math.pi
        x = x - dx

    for j in range(1, cols):
        x = ogm[0, c_row - 1, j]
        y = ogm[1, c_row - 1, j]
        th = ogm[2, c_row - 1, j]
        th_real = ogm[3, c_row - 1, j]

        for i in range(c_row - 1, 0):
            ogm[0, i, j] = x
            ogm[1, i, j] = y
            ogm[2, i, j] = th
            ogm[3, i, j] = th_real

        x = ogm[0, c_row - 1, j]
        y = ogm[1, c_row - 1, j]
        th = ogm[2, c_row - 1, j]
        th_real = ogm[3, c_row - 1, j]

        for i in range(c_row - 1, rows):
            ogm[0, i, j] = x
            ogm[1, i, j] = y
            ogm[2, i, j] = th
            ogm[3, i, j] = th_real
            x = x + dx * math.sin(th_real)
            y = y - dy * math.cos(th_real)

    plt.plot(ogm[0, c_row - 1, :], ogm[1, c_row - 1, :], 'o')
    plt.axis([0, cols, -60, 30])
    plt.show()
    return ogm


if __name__ == "__main__":
    environment_load()
