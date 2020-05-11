import numpy as np
import math


def curvature_fn(x, y):
    k = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for j in range(0, len(x)):
        k[j, 0] = (dx[j, 0] * ddy[j, 0] - dy[j, 0] * ddx[j, 0]) / (dx[j, 0] * dx[j, 0] + dy[j, 0] * dy[j, 0])

    return k


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
