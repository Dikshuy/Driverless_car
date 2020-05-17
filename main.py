import math
import numpy as np
import matplotlib.pyplot as plt


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
    for we in range(second_cols + 1, straight_cols2):
        lane[0, c_row - 1, we] = x + dx
        lane[1, c_row - 1, we] = y
        lane[2, c_row - 1, we] = math.pi
        lane[3, c_row - 1, we] = 0
        x = x + dx

    dth = math.pi / (third_cols - straight_cols2)
    th = math.pi / 2 + dth
    x = lane[0, c_row - 1, straight_cols2 - 1]
    y = lane[1, c_row - 1, straight_cols2 - 1]
    for we in range(straight_cols2 + 1, third_cols):
        lane[0, c_row - 1, we] = x - r0 * math.cos(th)
        lane[1, c_row - 1, we] = y - r0 * (1 - math.sin(th))
        lane[2, c_row - 1, we] = th
        lane[3, c_row - 1, we] = 0.5 * math.pi - th
        th = th + dth

    x = lane[0, c_row - 1, third_cols - 1]
    y = lane[1, c_row - 1, third_cols - 1]
    for we in range(third_cols + 1, cols):
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

        for we in range(c_row - 1, 0):
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

    plt.plot(lane[0, c_row - 1, :], lane[1, c_row - 1, :], 'o')
    plt.axis([0, cols, -60, 30])
    # plt.show()
    return lane


def curvature_fn(x, y):
    k_matrix = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for kl in range(0, len(x)):
        k_matrix[kl, 0] = (dx[kl, 0] * ddy[kl, 0] - dy[kl, 0] * ddx[kl, 0]) / (
                dx[kl, 0] * dx[kl, 0] + dy[kl, 0] * dy[kl, 0])

    return k_matrix


def vel_prof(x, y, v0, a0, t0, current_state):
    global v_obs, V_max, critical_dis, follow_dis, dyn_x, dyn_y, dyn1_x, dyn1_y
    length_of_array_x = len(x)

    vel_matrix = np.zeros(length_of_array_x, 1)
    longitudinal_acceleration = np.zeros(length_of_array_x, 1)
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

    for o in range(0, length_of_array_x):
        if c[o] != 0:  # turning is required
            v_all = min(v_max, math.sqrt(a_lat_max / abs(c[o])))
            temp = math.sqrt(
                (vel_matrix[o - 1] ** 2) + 2 * acc_long * math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2))
            vel_matrix[o] = max(min(temp, v_all), 0)  # ensure non-negative value of velocity

            temp1 = ((vel_matrix[o] ** 2) - (vel_matrix[o - 1] ** 2)) / (
                    2 * math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2))
            longitudinal_acceleration[o] = min(temp1, acc_long)

            wt[0] = v0 * c[o]
            wt[o] = vel_matrix[o] * c[o]

            if vel_matrix[o] == vel_matrix[o - 1] and vel_matrix[o] == 0:
                current_time[o] = current_time[o - 1] + 1

            elif vel_matrix[o] == vel_matrix[o - 1] and vel_matrix[o] != 0:
                current_time[o] = current_time[o - 1] + (math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2)) / \
                                  vel_matrix[o - 1]

            else:
                current_time[o] = current_time[o - 1] + (vel_matrix[o] - vel_matrix[o - 1]) / longitudinal_acceleration[
                    o]

        else:
            temp = math.sqrt(
                (vel_matrix[o - 1] ** 2) + 2 * acc_long * math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2))
            vel_matrix[o] = max(min(temp, v_max), 0)

            temp1 = ((vel_matrix[o] ** 2) - (vel_matrix[o - 1] ** 2)) / (
                    2 * math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2))
            longitudinal_acceleration[o] = min(temp1, acc_long)

            wt[0] = 0
            wt[o] = 0

            if vel_matrix[o] == vel_matrix[o - 1] and vel_matrix[o] == 0:
                current_time[o] = current_time[o - 1] + 1

            elif vel_matrix[o] == vel_matrix[o - 1] and vel_matrix[o] != 0:
                current_time[o] = current_time[o - 1] + (math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2)) / \
                                  vel_matrix[o - 1]

            else:
                current_time[o] = current_time[o - 1] + (vel_matrix[o] - vel_matrix[o - 1]) / longitudinal_acceleration[
                    o]

    vel_matrix[0] = v0
    longitudinal_acceleration[0] = a0
    current_time[0] = t0

    return x, y, vel_matrix, longitudinal_acceleration, wt, current_time


def overtake_1(a_store, b_store, store_x, store_y, T_initial, rr, tang_m, perp_m, index_number):
    global T_fin, theta4_n, ad_n, vd_n, wd_n, trajectory
    trajectory = 5
    num = 10
    td_n = T_initial

    for tr in range(0, trajectory):
        iterate = 0
        while iterate == 0 and tr == rr:
            if tang_m == 0:
                p = [[a_store[5, tr]]
                     [a_store[4, tr]]
                     [a_store[3, tr]]
                     [a_store[2, tr]]
                     [a_store[1, tr]]
                     [a_store[0, tr] - xq[index_number + 1]]]
                time = np.roots(p)
                for aa in range(0, len(time)):
                    if ~any(np.imag(time[aa])) == 1:
                        if time[aa] > 0:
                            T_fin = time[aa]

            else:
                p = [[b_store[5, tr] - perp_m * a_store[6, tr]],
                     [b_store[4, tr] - perp_m * a_store(5, tr)]
                     [b_store[3, tr] - perp_m * a_store(4, tr)]
                     [b_store[2, tr] - perp_m * a_store(3, tr)]
                     [b_store[1, tr] - perp_m * a_store(2, tr)]
                     [b_store[0, tr] - perp_m * a_store(1, tr) + perp_m * xq[index_number + 1] - yq[index_number + 1]]]
                time = np.roots(p)
                for aa in range(0, len(time)):
                    if ~any(np.imag(time[aa])) == 1:
                        if time[aa] > 0:
                            T_fin = time[aa]

            for ii in range(0, num):
                xd[ii][0] = [1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5] * a_store[:, tr]
                yd[ii][0] = [1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5] * b_store[:, tr]

                td_n = td_n + (T_fin - T_initial) / num

            curv = curvature_fn(store_x, store_y)
            A = np.array([0, 1, 2 * T_fin, 3 * T_fin ** 2, 4 * T_fin ** 3, 5 * T_fin ** 4])
            B = np.dot(A, a_store)
            C = np.dot(A, b_store)
            vd_n = max(min(math.sqrt(B * B + C * C), V_max), 0)
            wd_n = vd_n * curv[num + 1][0]
            D = np.array([0, 0, 2, 6 * T_fin, 12 * T_fin ^ 2, 20 * T_fin ^ 3])
            E = np.dot(D, a_store)
            F = np.dot(E, b_store)
            ad_n = min(math.sqrt(F * F + E * E), 1.2)

            for jj in range(0, num):
                if xq[index_number + 1] - xq[index_number] < 0:
                    theta4[jj][0] = math.pi + math.atan((yd[jj + 1] - yd[jj]) / xd[jj + 1] - xd[jj])
                else:
                    math.atan((yd[jj + 1] - yd[jj]) / xd[jj + 1] - xd[jj])

            theta4_n = theta4[num][0]

            store_x[:, tr] = xd
            store_y[:, tr] = yd
            iterate = iterate + 1

    return store_x, store_y, vd_n, ad_n, wd_n, theta4_n, index_number


def initialization1(velocity, acceleration_in_x, z, time, p, indexes, angle):
    global xq, yq, trajectory
    trajectory = 1

    v_i = velocity[p]
    v_f = velocity[p + 1]
    t_i = 0
    t_f = time[p + 1] - time[p]
    w_i = z[p]
    w_f = z[p + 1]
    a_i = acceleration_in_x[p]
    a_f = acceleration_in_x[p + 1]

    m_tang2 = (yq[indexes + 2] - yq[indexes + 1]) / (xq[indexes + 2] - xq[indexes + 1])
    if xq[indexes + 1] - xq[indexes] < 0:
        theta2 = math.pi + math.atan(m_tang2)
    else:
        theta2 = math.atan(m_tang2)

    P_matrix = [[1, t_i, t_i ** 2, t_i ** 3, t_i ** 4, t_i ** 5],
                [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
                [0, 0, 2, 6 * t_i, 12 * (t_i ** 2), 20 * (t_i ** 3)],
                [1, t_f, t_f ** 2, t_f ** 3, t_f ** 4, t_f ** 5],
                [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
                [0, 0, 2, 6 * t_f, 12 * (t_f ** 2), 20 * (t_f ** 3)]]

    vx_i = v_i * math.cos(angle)
    vx_f = v_f * math.cos(theta2)
    vy_i = v_i * math.sin(angle)
    vy_f = v_f * math.sin(theta2)
    ax_i = a_i * math.cos(angle) - v_i * math.sin(angle) * w_i
    ax_f = a_f * math.cos(theta2) - v_f * math.sin(theta2) * w_f
    ay_i = a_i * math.sin(angle) + v_i * math.cos(angle) * w_i
    ay_f = a_f * math.sin(theta2) + v_f * math.cos(theta2) * w_f

    set_x = xq[indexes + 1]
    set_y = yq[indexes + 1]

    return vx_i, vx_f, vy_i, vy_f, ax_i, ax_f, ay_i, ay_f, t_i, t_f, v_f, a_f, set_x, set_y, P_matrix, trajectory


def cost(xd1, yd1, x_centre, y_centre, phase_chosen):
    global total_cost
    if max(curvature_fn(xd1, yd1)) <= 0.15:
        if phase_chosen == 2:
            cost1 = 0
            cost2 = np.sum(math.sqrt(np.square(x_centre - xd1) + np.square(y_centre - yd1)))
            total_cost = 2 * cost1 + 0.5 * cost2
        else:
            cost1 = 1 / max(min(math.sqrt(np.square(xd1 - dyn1_x) + np.square(yd1 - dyn1_y))) - 2, 0.01)
            cost2 = np.sum(math.sqrt(np.square(x_centre - xd1) + np.square(y_centre - yd1)))
            total_cost = 2 * cost1 + 0.5 * cost2

    else:
        total_cost = math.inf

    return total_cost


ogm = environment_load()
xq = ogm[0, 4, :]
yq = ogm[1, 4, :]

x_set_init = 0
y_set_init = 2
# T = []
# V = []
V_init = 0
t_init = 0
A_init = 1
T = t_init
V = V_init

# id = 23
id2 = 41

lane_status = 0

# variable definition

T_fin = []
xd_n = []
yd_n = []
theta4_n = []
g1 = []
g2 = []
x_store = []
y_store = []
a_coefficient_store = []
b_coefficient_store = []
time_store = []
cost_store = []
num_foc_t = 0
v_obs = 1.5
V_max = 5
horizon = 5
index = 1
state = 0
prev_state = 0
phase = 0
critical_dis = 10
dyn_x = 0
dyn_y = 0
dyn1_x = 31.3
dyn1_y = 2
total_cost = 0

follow_dis = critical_dis
trajectory = 1
itr2 = 0
vd_n = 0
wd_n = 0
ad_n = 0
theta4 = 0
a = 0

while index < len(xq):
    global x_set, y_set, Ax_f, Ax_i, Ay_f, Ay_i, Vx_f, Vx_i, Vy_f, Vy_i, P, T_i, T_f, V_f
    if state == 5:
        state = prev_state
        [xr, yr, vel, a_long, w, t] = vel_prof(xq[index:index + horizon], yq[index: index + horizon], V_init, A_init,
                                               t_init, state)
        i = 1
        tempor = prev_state
        prev_state = state
        itr = 0

        while state == prev_state:
            m_tang = (yq[index + 1] - yq[index]) / (xq[index + 1] - xq[index])
            m_perp = -1 / m_tang

            xNormalLine = (1 / m_perp) * [yq - yq[index + 1]] + xq[index + 1]
            if xq[index + 1] - xq[index] < 0:
                theta = math.pi + math.atan(m_tang)
            else:
                theta = math.atan(m_tang)

            if state != 2 and phase == 0:
                (Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P,
                 trajectory) = initialization1(vel, a_long, w, t, i, index, theta)

            # elif state == 2 and phase == 2:
            #     [Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P,
            #      traj] = initialization_3_5_traj(vel, a_long, w, t, i, index, theta)
            #
            # elif state ==2 and phase ==1:
            #     [Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, traj, itr2,
            #      m_perp3] = initialization_2(vel, a_long, w, i, dist, itr2, vd_n, wd_n, ad_n, index, theta, theta4)
            #     xNormalLine1 = (1 / m_perp3) * (yq - dyn1_y) + dyn1_x

            for h in range(1, trajectory):
                xt_f = x_set[h, 0]
                yt_f = y_set[h, 0]

                if state == 2 and phase == 2:
                    g1[:, h] = [[x_set_init], [Vx_i], [Ax_i], xt_f[h, 0], Vx_f[h, 0], Ax_f[h, 0]]
                    g2[:, h] = [[y_set_init], [Vy_i], [Ay_i], yt_f[h, 0], Vy_f[h, 0], Ay_f[h, 0]]

                    a_coeff = np.linalg.lstsq(P[:, :, h], g1[:, h])
                    b_coeff = np.linalg.lstsq(P[:, :, h], g2[:, h])

                else:
                    g1 = [[x_set_init], Vx_i, Ax_i, xt_f, Vx_f, Ax_f]
                    g2 = [[x_set_init], Vy_i, Ay_i, yt_f, Vy_f, Ay_f]

                    a_coeff = np.linalg.lstsq[P, g1]
                    b_coeff = np.linalg.lstsq[P, g2]

                num_foc_t = 10
                xd = np.zeros((num_foc_t + 1, 1))
                yd = np.zeros((num_foc_t + 1, 1))
                xd_o = np.zeros((num_foc_t + 1, 1))
                yd_0 = np.zeros((num_foc_t + 1, 1))
                x_cen = np.zeros((num_foc_t + 1, 1))
                y_cen = np.zeros((num_foc_t + 1, 1))
                TD = np.zeros((num_foc_t + 1, 1))
                vd = np.zeros((num_foc_t + 1, 1))
                wd = np.zeros((num_foc_t + 1, 1))
                td = T_i
                xpoint = np.zeros((num_foc_t + 1, 1))
                ypoint = np.zeros((num_foc_t + 1, 1))
                xpnt = np.zeros((num_foc_t + 1, 1))
                ypnt = np.zeros((num_foc_t + 1, 1))

                for f in range(1, num_foc_t + 1):
                    xd[f, 1] = [[1], [td], [td ** 2], [td ** 3], [td ** 4], [td ** 5]] * a_coeff
                    yd[f, 1] = [[1], [td], [td ** 2], [td ** 3], [td ** 4], [td ** 5]] * b_coeff

                    td_old = td
                    if state == 2 and phase == 2:
                        td = td + (T_f[h, 1] - T_i) / num_foc_t
                    else:
                        td = td + (T_f - T_i) / num_foc_t

                    TD[f, 0] = td

                    x_cen[f, 0] = xq[index, 0] + f * ((xq[index + 1, 0] - xq[index, 0]) / num_foc_t)
                    y_cen[f, 1] = yq[index, 1] + f * ((yq[index + 1, 0] - yq[index, 0]) / num_foc_t)

                x_store[:, h] = xd
                y_store[:, h] = yd

                a_coefficient_store[:, h] = a_coeff
                b_coefficient_store[:, h] = b_coeff
                time_store[:, h] = TD

                d = np.argmin(np.abs(xq - dyn1_x))
                if m_tang == math.inf:
                    if yq[d] <= dyn1_y:
                        j = d + 1
                    elif yq[d] > dyn1_y:
                        j = d
                else:
                    if xq[d] <= dyn1_x:
                        j = d + 1
                    elif xq[d] > dyn1_x:
                        j = d

                for q in range(1, num_foc_t + 1):
                    if index <= j and itr1 == 0:
                        if math.sqrt((xd[q, 0] - dyn1_x) ** 2 + (yd[q, 0] - dyn1_y) ** 2) < 10:
                            if lane_status == 0:
                                phase = 1
                                state = 2
                                xpnt[q, 1] = xd[q, 1]
                                ypnt[q, 1] = yd[q, 1]
                                s = np.where(xpnt)
                                itr2 = itr2 + 1
                                if itr2 == 1:
                                    x_store[1:s, h] = xd[1:s, 1]
                                    y_store[1:s, h] = yd[1:s, 1]
                                    x_store[s + 1:num_foc_t, h] = xd[s, 1]
                                    y_store[s + 1:num_foc_t, h] = yd[s, 1]
                                    break
                                elif itr2 > 1:
                                    total_cost = cost(xd, yd, x_cen, y_cen, phase)
                                    break

                            elif lane_status == 1:
                                state = 1

                        else:
                            phase = 0
                            state = 0

                    else:
                        if phase == 2:
                            state = 2
                            total_cost = cost(xd, yd, x_cen, y_cen, phase)
                            break
                        elif phase == 0:
                            state = 0

                if itr2 > 1 or phase == 2:
                    cost_store[h, 0] = total_cost
                    break

            if itr2 > 1 or phase == 2:
                if cost_store[:, :] == math.inf:
                    k = 5
                else:
                    # [ , k] = min(cost_store)
                    break

                [x_store, y_store, vd_n, ad_n, wd_n, theta4, index] = overtake_1(a_coefficient_store,
                                                                                 b_coefficient_store, x_store,
                                                                                 y_store, T_i, k, m_tang, m_perp, index)

            if state == 2 and itr2 > 1:
                if phase == 1:
                    x_set_init = x_store(num_foc_t + 1, k)
                    y_set_init = y_store(num_foc_t + 1, k)
                    dist = math.sqrt((x_set_init - dyn1_x) ** 2 + (y_set_init - dyn1_y) ** 2)

                    if m_tang == 0:
                        if x_store(num_foc_t + 1, k) > dyn1_x:
                            phase = 2
                            itr1 = itr1 + 1

                    elif m_tang == math.inf:
                        if y_store(num_foc_t + 1, k) > dyn1_y:
                            phase = 2
                            itr1 = itr1 + 1
                    elif m_tang != 0:
                        if xq[index + 1] - xq[index] < 0 or yq[index + 1] - yq[index] < 0:
                            if x_store(num_foc_t + 1, k) < dyn1_x and y_store(num_foc_t + 1, k) < dyn1_y:
                                phase = 2
                                itr1 = itr1 + 1
                    else:
                        if x_store(num_foc_t + 1, k) > dyn1_x:
                            phase = 2
                            itr1 = itr1 + 1
            elif phase == 2:
                x_set_init = x_store(num_foc_t + 1, k)
                y_set_init = y_store(num_foc_t + 1, k)
                dist = math.sqrt((x_set_init - dyn1_x) ** 2 + (y_set_init - dyn1_y) ** 2)

                if x_store(num_foc_t + 1, k) == xq[index + 1] and y_store(num_foc_t + 1, k) == yq[index + 1]:
                    phase = 0
                    state = 0
                    itr1 = itr1 + 1

        else:
            x_set_init = x_store(num_foc_t + 1, 1)
            y_set_init = y_store(num_foc_t + 1, 1)
            dist = math.sqrt((x_set_init - dyn1_x) ** 2 + (y_set_init - dyn1_y) ** 2)

        # plots

        plt.plot(ogm[0, 2, :], ogm[1, 2, :], linewidth=2, c='blue')
        plt.plot(ogm[0, 4, :], ogm[1, 4, :], c='red')
        plt.plot(ogm[0, 6, :], ogm[1, 6, :], linestyle='--')
        plt.plot(ogm[0, 10, :], ogm[1, 10, :], linewidth=2, c='blue')
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.axis([0, 165, -35, 35])

        for b in range(0, trajectory):
            plt.plot(x_store[:, b], y_store[:, b], linestyle='-', linewidth=1, c='blue')

        if state == 2 and itr2 > 1:
            plt.plot(x_store[:, k], y_store[:, k], linestyle='-', linewidth=2, c='blue')
            # plt.plot(xNormalLine1, yq, 'g-')

            if phase == 1:
                for r in range(0, num_foc_t + 1):
                    plt.plot(x_store(r, k), y_store(r, k), 'or', markersize=6)
                    plt.plot(dyn1_x, dyn1_y, 'sk', markersize=10, markerfacecolor='magenta')
                    plt.title('Detected state = ', str(state))
                    plt.pause(0.001)

            elif phase == 2:
                for r in range(0, num_foc_t + 1):
                    plt.plot(x_store(r, k), y_store(r, k), 'or', markersize=6)
                    plt.plot(dyn1_x, dyn1_y, 'sk', markerSize=10, markerFaceColor='magenta')
                    plt.title(['Detected state = ', str(state)])
                    plt.pause(0.01)

            else:
                plt.plot(x_store[:, 0], y_store[:, 0], linestyle='-', linewidth='2', color='red')
                for r in range(0, num_foc_t + 1):
                    plt.plot(x_store(r, 1), y_store(r, 1), 'or', markersize=6)
                    plt.plot(dyn1_x, dyn1_y, 'sk', markerSize=10, markerFaceColor='magenta')
                    plt.title(['Detected state = ', str(state)])
                    plt.pause(0.01)

        t_init = t[i + 1]
        if state != prev_state:
            if phase == 0 and itr != 0:
                V_init = vd_n
                A_init = ad_n
                itr1 = 0

            itr2 = 0
            break
        if i >= horizon:
            state = 5

        index = index + 1
        i = i + 1
        if phase == 2 and itr1 != 0:
            V_init = vd_n
            A_init = ad_n
        else:
            V_init = V_f
            A_init = A_f
