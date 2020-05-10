import math
import numpy as np
import sympy as sym
import matplotlib.pyplot as plt


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
    # plt.show()
    return ogm


v_obs = 0
V_max = 0
critical_dis = 0
follow_dis = 0
dyn_x = 0
dyn_y = 0
dyn1_x = 0
dyn1_y = 0


def vel_prof(x, y, v0, a0, t0, state):
    global v_obs, V_max, critical_dis, follow_dis, dyn_x, dyn_y, dyn1_x, dyn1_y
    length_of_array_x = len(x)

    vel = np.zeros(length_of_array_x, 1)
    a_long = np.zeros(length_of_array_x, 1)
    w = np.zeros(length_of_array_x, 1)
    t = np.zeros(length_of_array_x, 1)
    c = curvature_fn(x, y)
    a_lat_max = 0.2
    a_long_max = 0.5
    vel[0] = v0
    t[0] = t0
    a_long[0] = a0

    def zero():
        v_max = V_max
        acc_long = a_long_max
        return v_max, acc_long

    def one():
        v_max = min(v_obs, V_max)
        acc = 0.5 * (v_max ** 2 - v0 ** 2) / (math.sqrt((x[0] - dyn_x) ** 2 + (y[0] - dyn_y) ** 2) - follow_dis)
        acc_long = min(acc, a_long_max)
        return v_max, acc_long

    def two():
        v_max = V_max
        acc_long = a_long_max
        return v_max, acc_long

    def three():
        dis = math.sqrt((x[0] - dyn1_x) ** 2 + (y[0] - dyn1_y) ** 2) - critical_dis
        v_max = 0.0
        acc_long = max(-v0 ** 2 / (2 * abs(dis)), -V_max)
        return v_max, acc_long

    def five():
        v_max = V_max
        acc_long = a_long_max
        return v_max, acc_long

    def number(argument):
        switcher = {
            0: zero,
            1: one,
            2: two,
            3: three,
            5: five
        }
        func = switcher.get(argument, "Invalid month")
        return func()

    v_max, acc_long = number(state)

    for i in range(0, length_of_array_x):
        if c[i] != 0:
            v_all = min(v_max, math.sqrt(a_lat_max / abs(c[i])))
            temp = math.sqrt((vel[i - 1] * vel[i - 1]) + 2 * acc_long * math.sqrt(
                (x[i - 1] - x[i])(x[i - 1] - x[i]) + (y[i - 1] - y[i]) * (y[i - 1] - y[i])))
            vel[i] = max(min(temp, v_all), 0)

            temp1 = ((vel[i] * vel[i]) - (vel[i - 1] * vel[i - 1])) / (
                    2 * math.sqrt((x[i - 1] - x[i])(x[i - 1] - x[i]) + (y[i - 1] - y[i]) * (y[i - 1] - y[i])))
            a_long[i] = min(temp1, acc_long)

            w[0] = v0 * c[i]
            w[i] = vel[i] * c[i]

            if vel[i] == vel[i - 1] and vel[i] == 0:
                t[i] = t[i - 1] + 1

            elif vel[i] == vel[i - 1] and vel[i] != 0:
                t[i] = t[i - 1] + (math.sqrt((x[i - 1] - x[i])(x[i - 1] - x[i]) + (y[i - 1] - y[i])(y[i - 1] - y[i]))) / \
                       vel[i - 1]

            else:
                t[i] = t[i - 1] + (vel[i] - vel[i - 1]) / a_long[i]

        else:
            temp = math.sqrt((vel[i - 1] * vel[i - 1]) + 2 * acc_long * math.sqrt(
                (x[i - 1] - x[i]) * (x[i - 1] - x[i]) + (y[i - 1] - y[i]) * (y[i - 1] - y[i])))
            vel[i] = max(min(temp, v_max), 0)

            temp1 = (vel[i] * vel[i] - vel[i - 1] * vel[i - 1]) / (
                    2 * math.sqrt((x[i - 1] - x[i]) * (x[i - 1] - x[i]) + (y[i - 1] - y[i]) * (y[i - 1] - y[i])))
            a_long[i] = min(temp1, acc_long)

            w[0] = 0
            w[i] = 0

            if vel[i] == vel[i - 1] and vel[i] == 0:
                t[i] = t[i - 1] + 1

            elif vel[i] == vel[i - 1] and vel[i] != 0:
                t[i] = t[i - 1] + (math.sqrt((x[i - 1] - x[i])(x[i - 1] - x[i]) + (y[i - 1] - y[i])(y[i - 1] - y[i]))) / \
                       vel[i - 1]

            else:
                t[i] = t[i - 1] + (vel[i] - vel[i - 1]) / a_long[i]

    vel[0] = v0
    a_long[0] = a0
    t[0] = t0

    return x, y, vel, a_long, w, t


def curvature_fn(x, y):
    k = np.zeros(len(x), 1)
    dx = sym.diff(x)
    ddx = sym.diff(dx)
    dy = sym.diff(y)
    ddy = sym.diff(dy)

    for j in range(0, len(x)):
        k[j, 0] = (dx(j, 0) * ddy(j, 0) - dy(j, 0) * ddx(j, 0)) / (dx(j, 0) * dx(j, 0) + dy(j, 0) * dy(j, 0))

    return k


xq = []
yq = []


def initialization1(vel, a_long, w, t, i, index, theta):
    global xq, yq
    traj = 1

    v_i = vel[i]
    v_f = vel[i + 1]
    t_i = 0
    t_f = t[i + 1] - t[i]
    w_i = w[i]
    w_f = w[i + 1]
    a_i = a_long[i]
    a_f = a_long[i + 1]

    m_tang2 = (yq[index + 2] - yq[index + 1]) / (xq[index + 2] - xq[index + 1])
    if xq[index + 1] - xq[index] < 0:
        theta2 = math.pi + math.atan(m_tang2)
    else:
        theta2 = math.atan(m_tang2)

    P = [[1, t_i, t_i ** 2, t_i ** 3, t_i ** 4, t_i ** 5],
         [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
         [0, 0, 2, 6 * t_i, 12 * (t_i ** 2), 20 * (t_i ** 3)],
         [1, t_f, t_f ** 2, t_f ** 3, t_f ** 4, t_f ** 5],
         [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
         [0, 0, 2, 6 * t_f, 12 * (t_f ** 2), 20 * (t_f ** 3)]]

    vx_i = v_i * math.cos(theta)
    vx_f = v_f * math.cos(theta2)
    vy_i = v_i * math.sin(theta)
    vy_f = v_f * math.sin(theta2)
    ax_i = a_i * math.cos(theta) - v_i * math.sin(theta) * w_i
    ax_f = a_f * math.cos(theta2) - v_f * math.sin(theta2) * w_f
    ay_i = a_i * math.sin(theta) + v_i * math.cos(theta) * w_i
    ay_f = a_f * math.sin(theta2) + v_f * math.cos(theta2) * w_f

    x_set = xq[index + 1]
    y_set = yq[index + 1]

    return vx_i, vx_f, vy_i, vy_f, ax_i, ax_f, ay_i, ay_f, t_i, t_f, v_f, a_f, x_set, y_set, P, traj
