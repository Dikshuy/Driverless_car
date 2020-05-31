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


def vel_prof(x, y, v0, a0, t0, current_state):
    global v_obs, V_max, critical_dis, follow_dis, dyn_x, dyn_y, dyn1_x, dyn1_y
    length_of_array_x = len(x)

    vel_matrix = np.zeros((length_of_array_x, 1))
    longitudinal_acceleration = np.zeros((length_of_array_x, 1))
    wt = np.zeros(length_of_array_x, 1)
    current_time = np.zeros(length_of_array_x, 1)
    c = curvature_fn(x, y)
    a_lat_max = 0.2
    a_long_max = 0.5
    vel_matrix[0] = v0
    current_time[0] = t0
    longitudinal_acceleration[0] = a0

    def zero():  # clear road ahead
        max_v = V_max
        long_acc = a_long_max
        return max_v, long_acc

    def one():  # follow the vehicle ahead in same lane
        max_v = min(v_obs, V_max)
        acc = 0.5 * (max_v ** 2 - v0 ** 2) / (math.sqrt((x[0] - dyn_x) ** 2 + (y[0] - dyn_y) ** 2) - follow_dis)
        long_acc = min(acc, a_long_max)
        return max_v, long_acc

    def two():  # overtake the vehicle ahead
        max_v = V_max
        long_acc = a_long_max
        return max_v, long_acc

    def three():  # emergency braking
        dis = math.sqrt((x[0] - dyn1_x) ** 2 + ((y[0] - dyn1_y) ** 2) - critical_dis)
        max_v = 0.0
        long_acc = max(-v0 ** 2 / (2 * abs(dis)), -V_max)
        return max_v, long_acc

    def five():  # horizon exceeded
        max_v = V_max
        long_acc = a_long_max
        return max_v, long_acc

    def number(argument):  # defining switch function
        switcher = {
            0: zero,
            1: one,
            2: two,
            3: three,
            5: five
        }
        func = switcher.get(argument, "Invalid state")
        return func()

    v_max, acc_long = number(current_state)

    for b in range(0, length_of_array_x):
        if c[b] != 0:  # turning is required
            v_all = min(v_max, math.sqrt(a_lat_max / abs(c[b])))
            temp = math.sqrt(
                (vel_matrix[b - 1] ** 2) + 2 * acc_long * math.sqrt((x[b - 1] - x[b]) ** 2 + (y[b - 1] - y[b]) ** 2))
            vel_matrix[b] = max(min(temp, v_all), 0)  # ensure non-negative value of velocity

            temp1 = ((vel_matrix[b] ** 2) - (vel_matrix[b - 1] ** 2)) / (
                        2 * math.sqrt((x[b - 1] - x[b]) ** 2 + (y[b - 1] - y[b]) ** 2))
            longitudinal_acceleration[b] = min(temp1, acc_long)

            wt[0] = v0 * c[b]
            wt[b] = vel_matrix[b] * c[b]

            if vel_matrix[b] == vel_matrix[b - 1] and vel_matrix[b] == 0:
                current_time[b] = current_time[b - 1] + 1

            elif vel_matrix[b] == vel_matrix[b - 1] and vel_matrix[b] != 0:
                current_time[b] = current_time[b - 1] + (math.sqrt((x[b - 1] - x[b]) ** 2 + (y[b - 1] - y[b]) ** 2)) / \
                                  vel_matrix[b - 1]

            else:
                current_time[b] = current_time[b - 1] + (vel_matrix[b] - vel_matrix[b - 1]) / longitudinal_acceleration[
                    b]

        else:
            temp = math.sqrt(
                (vel_matrix[b - 1] ** 2) + 2 * acc_long * math.sqrt((x[b - 1] - x[b]) ** 2 + (y[b - 1] - y[b]) ** 2))
            vel_matrix[b] = max(min(temp, v_max), 0)

            temp1 = ((vel_matrix[b] ** 2) - (vel_matrix[b - 1] ** 2)) / (
                        2 * math.sqrt((x[b - 1] - x[b]) ** 2 + (y[b - 1] - y[b]) ** 2))
            longitudinal_acceleration[b] = min(temp1, acc_long)

            wt[0] = 0
            wt[b] = 0

            if vel_matrix[b] == vel_matrix[b - 1] and vel_matrix[b] == 0:
                current_time[b] = current_time[b - 1] + 1

            elif vel_matrix[b] == vel_matrix[b - 1] and vel_matrix[b] != 0:
                current_time[b] = current_time[b - 1] + (math.sqrt((x[b - 1] - x[b]) ** 2 + (y[b - 1] - y[b]) ** 2)) / \
                                  vel_matrix[b - 1]

            else:
                current_time[b] = current_time[b - 1] + (vel_matrix[b] - vel_matrix[b - 1]) / longitudinal_acceleration[
                    b]

    vel_matrix[0] = v0
    longitudinal_acceleration[0] = a0
    current_time[0] = t0

    return x, y, vel_matrix, longitudinal_acceleration, wt, current_time
