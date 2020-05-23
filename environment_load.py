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

    lane = np.zeros((4, rows, cols))

    dx = 1.0
    dy = 1.0

    c_row = 6
    x = 0
    y = 0

    for we in range(0, str_cols):
        lane[0, c_row - 1, we] = x
        lane[1, c_row - 1, we] = y
        lane[2, c_row - 1, we] = 0
        lane[3, c_row - 1, we] = 0
        x = x + dx

    r0 = 20
    dth = 0.25 * math.pi / (first_cols - str_cols)
    x = lane[0, c_row - 1, str_cols - 1]
    y = lane[1, c_row - 1, str_cols - 1]
    th = dth

    for we in range(str_cols, first_cols):
        lane[0, c_row - 1, we] = x + r0 * math.sin(th)
        lane[1, c_row - 1, we] = y + r0 * (1 - math.cos(th))
        lane[2, c_row - 1, we] = th
        lane[3, c_row - 1, we] = th
        th = th + dth

    dth = 0.25 * math.pi / (second_cols - first_cols)
    th = 0.25 * math.pi + dth
    x = lane[0, c_row - 1, first_cols - 1]
    y = lane[1, c_row - 1, first_cols - 1]

    for we in range(first_cols, second_cols):
        lane[0, c_row - 1, we] = x + r0 * math.cos(0.25 * math.pi) - r0 * math.cos(th)
        lane[1, c_row - 1, we] = y + r0 * math.sin(th) - r0 * math.sin(0.25 * math.pi)
        lane[2, c_row - 1, we] = 1.5 * math.pi - th
        lane[3, c_row - 1, we] = 0.5 * math.pi - th
        th = th + dth

    x = lane[0, c_row - 1, second_cols - 1]
    y = lane[1, c_row - 1, second_cols - 1]
    for we in range(second_cols, straight_cols2):
        lane[0, c_row - 1, we] = x + dx
        lane[1, c_row - 1, we] = y
        lane[2, c_row - 1, we] = math.pi
        lane[3, c_row - 1, we] = 0
        x = x + dx

    dth = math.pi / (third_cols - straight_cols2)
    th = math.pi / 2 + dth
    x = lane[0, c_row - 1, straight_cols2 - 1]
    y = lane[1, c_row - 1, straight_cols2 - 1]
    for we in range(straight_cols2, third_cols):
        lane[0, c_row - 1, we] = x - r0 * math.cos(th)
        lane[1, c_row - 1, we] = y - r0 * (1 - math.sin(th))
        lane[2, c_row - 1, we] = th
        lane[3, c_row - 1, we] = 0.5 * math.pi - th
        th = th + dth

    x = lane[0, c_row - 1, third_cols - 1]
    y = lane[1, c_row - 1, third_cols - 1]
    for we in range(third_cols, cols):
        lane[0, c_row - 1, we] = x - dx
        lane[1, c_row - 1, we] = y
        lane[2, c_row - 1, we] = 0
        lane[3, c_row - 1, we] = math.pi
        x = x - dx

    for c in range(1, cols):
        x = lane[0, c_row - 1, c]
        y = lane[1, c_row - 1, c]
        th = lane[2, c_row - 1, c]
        th_real = lane[3, c_row - 1, c]

        for we in range(0, c_row-1):
            lane[0, we, c] = x
            lane[1, we, c] = y
            lane[2, we, c] = th
            lane[3, we, c] = th_real

        x = lane[0, c_row - 1, c]
        y = lane[1, c_row - 1, c]
        th = lane[2, c_row - 1, c]
        th_real = lane[3, c_row - 1, c]

        for we in range(c_row - 1, rows):
            lane[0, we, c] = x
            lane[1, we, c] = y
            lane[2, we, c] = th
            lane[3, we, c] = th_real
            x = x + dx * math.sin(th_real)
            y = y - dy * math.cos(th_real)

    plt.plot(lane[0, 0, :], lane[1, 0, :], linewidth=2, c='black')
    # print(lane[0, c_row - 3, :])
    # print(lane[0, c_row - 1, :])
    # plt.axis([0, cols, -60, 30])
    # print(lane)
    return lane


ogm = environment_load()
xq = ogm[0, 3, :]
yq = ogm[1, 3, :]
print(len(xq))
plt.plot(ogm[0, 0, :], ogm[1, 0, :], linewidth=2, c='black')
# plt.plot(ogm[0, 4, :], ogm[1, 4, :], c='orange', linewidth=1)
plt.plot(ogm[0, 6, :], ogm[1, 6, :], linestyle='--')
plt.plot(ogm[0, 10, :], ogm[1, 10, :], linewidth=2, c='black')

plt.xlabel('X(m)')
plt.ylabel('Y(m)')
plt.axis([0, 165, -35, 35])
print(ogm[0, 10, :])
print(ogm[0, 5, :])
# plt.show()
