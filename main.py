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

    for d in range(0, str_cols):
        lane[0, c_row - 1, d] = x
        lane[1, c_row - 1, d] = y
        lane[2, c_row - 1, d] = 0
        lane[3, c_row - 1, d] = 0

        x = x + dx

    r0 = 20
    dth = 0.25 * math.pi / (first_cols - str_cols)
    x = lane[0, c_row - 1, str_cols - 1]
    y = lane[1, c_row - 1, str_cols - 1]
    th = dth

    for d in range(str_cols, first_cols):
        lane[0, c_row - 1, d] = x + r0 * math.sin(th)
        lane[1, c_row - 1, d] = y + r0 * (1 - math.cos(th))
        lane[2, c_row - 1, d] = th
        lane[3, c_row - 1, d] = th
        th = th + dth

    for d in range(str_cols, first_cols):
        print(x + r0 * math.sin(th))
        th = th + dth

    dth = 0.25 * math.pi / (second_cols - first_cols)
    th = 0.25 * math.pi + dth
    x = lane[0, c_row - 1, first_cols - 1]
    y = lane[1, c_row - 1, first_cols - 1]

    for d in range(first_cols, second_cols):
        lane[0, c_row - 1, d] = x + r0 * math.cos(0.25 * math.pi) - r0 * math.cos(th)
        lane[1, c_row - 1, d] = y + r0 * math.sin(th) - r0 * math.sin(0.25 * math.pi)
        lane[2, c_row - 1, d] = 1.5 * math.pi - th
        lane[3, c_row - 1, d] = 0.5 * math.pi - th
        th = th + dth

    x = lane[0, c_row - 1, second_cols - 1]
    y = lane[1, c_row - 1, second_cols - 1]
    for d in range(second_cols + 1, straight_cols2):
        lane[0, c_row - 1, d] = x + dx
        lane[1, c_row - 1, d] = y
        lane[2, c_row - 1, d] = math.pi
        lane[3, c_row - 1, d] = 0
        x = x + dx

    dth = math.pi / (third_cols - straight_cols2)
    th = math.pi / 2 + dth
    x = lane[0, c_row - 1, straight_cols2 - 1]
    y = lane[1, c_row - 1, straight_cols2 - 1]
    for d in range(straight_cols2 + 1, third_cols):
        lane[0, c_row - 1, d] = x - r0 * math.cos(th)
        lane[1, c_row - 1, d] = y - r0 * (1 - math.sin(th))
        lane[2, c_row - 1, d] = th
        lane[3, c_row - 1, d] = 0.5 * math.pi - th
        th = th + dth

    x = lane[0, c_row - 1, third_cols - 1]
    y = lane[1, c_row - 1, third_cols - 1]
    for d in range(third_cols + 1, cols):
        lane[0, c_row - 1, d] = x - dx
        lane[1, c_row - 1, d] = y
        lane[2, c_row - 1, d] = 0
        lane[3, c_row - 1, d] = math.pi
        x = x - dx

    for j in range(1, cols):
        x = lane[0, c_row - 1, j]
        y = lane[1, c_row - 1, j]
        th = lane[2, c_row - 1, j]
        th_real = lane[3, c_row - 1, j]

        for d in range(c_row - 1, 0):
            lane[0, d, j] = x
            lane[1, d, j] = y
            lane[2, d, j] = th
            lane[3, d, j] = th_real

        x = lane[0, c_row - 1, j]
        y = lane[1, c_row - 1, j]
        th = lane[2, c_row - 1, j]
        th_real = lane[3, c_row - 1, j]

        for d in range(c_row - 1, rows):
            lane[0, d, j] = x
            lane[1, d, j] = y
            lane[2, d, j] = th
            lane[3, d, j] = th_real
            x = x + dx * math.sin(th_real)
            y = y - dy * math.cos(th_real)

    plt.plot(lane[0, c_row - 1, :], lane[1, c_row - 1, :], 'o')
    plt.axis([0, cols, -60, 30])
    # plt.show()
    return lane


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

    for b in range(0, length_of_array_x):
        if c[b] != 0:
            v_all = min(v_max, math.sqrt(a_lat_max / abs(c[b])))
            temp = math.sqrt((vel[b - 1] * vel[b - 1]) + 2 * acc_long * math.sqrt(
                (x[b - 1] - x[b])(x[b - 1] - x[b]) + (y[b - 1] - y[b]) * (y[b - 1] - y[b])))
            vel[b] = max(min(temp, v_all), 0)

            temp1 = ((vel[b] * vel[b]) - (vel[b - 1] * vel[b - 1])) / (
                    2 * math.sqrt((x[b - 1] - x[b])(x[b - 1] - x[b]) + (y[b - 1] - y[b]) * (y[b - 1] - y[b])))
            a_long[b] = min(temp1, acc_long)

            w[0] = v0 * c[b]
            w[b] = vel[b] * c[b]

            if vel[b] == vel[b - 1] and vel[b] == 0:
                t[b] = t[b - 1] + 1

            elif vel[b] == vel[b - 1] and vel[b] != 0:
                t[b] = t[b - 1] + (math.sqrt((x[b - 1] - x[b])(x[b - 1] - x[b]) + (y[b - 1] - y[b])(y[b - 1] - y[b]))) / \
                       vel[b - 1]

            else:
                t[b] = t[b - 1] + (vel[b] - vel[b - 1]) / a_long[b]

        else:
            temp = math.sqrt((vel[b - 1] * vel[b - 1]) + 2 * acc_long * math.sqrt(
                (x[b - 1] - x[b]) * (x[b - 1] - x[b]) + (y[b - 1] - y[b]) * (y[b - 1] - y[b])))
            vel[b] = max(min(temp, v_max), 0)

            temp1 = (vel[b] * vel[b] - vel[b - 1] * vel[b - 1]) / (
                    2 * math.sqrt((x[b - 1] - x[b]) * (x[b - 1] - x[b]) + (y[b - 1] - y[b]) * (y[b - 1] - y[b])))
            a_long[b] = min(temp1, acc_long)

            w[0] = 0
            w[b] = 0

            if vel[b] == vel[b - 1] and vel[b] == 0:
                t[b] = t[b - 1] + 1

            elif vel[b] == vel[b - 1] and vel[b] != 0:
                t[b] = t[b - 1] + (math.sqrt((x[b - 1] - x[b])(x[b - 1] - x[b]) + (y[b - 1] - y[b])(y[b - 1] - y[b]))) / \
                       vel[b - 1]

            else:
                t[b] = t[b - 1] + (vel[b] - vel[b - 1]) / a_long[b]

    vel[0] = v0
    a_long[0] = a0
    t[0] = t0

    return x, y, vel, a_long, w, t


def curvature_fn(x, y):
    k = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for j in range(0, len(x)):
        k[j, 0] = (dx[j, 0] * ddy[j, 0] - dy[j, 0] * ddx[j, 0]) / (dx[j, 0] * dx[j, 0] + dy[j, 0] * dy[j, 0])
    return k


def overtake_1(a_store, b_store, x_store, y_store, T_i, k, m_tang, m_perp, index):
    global T_fin, theta4_n, ad_n, vd_n, wd_n
    traj = 5
    num_foc_t = 10
    td_n = T_i

    for tr in range(0, traj):
        iterate = 0
        while iterate == 0 and tr == k:
            if m_tang == 0:
                p = [[a_store[6, tr]],
                     [a_store[5, tr]]
                     [a_store[4, tr]]
                     [a_store[3, tr]]
                     [a_store[2, tr]]
                     [a_store[1, tr] - xq[index + 1]]]
                time = np.roots(p)
                for j in range(0, len(time)):
                    if ~any(np.imag(time[j])) == 1:
                        if time[j] > 0:
                            T_fin = time[j]

            else:
                p = [[b_store[6, tr] - m_perp * a_store[6, tr]],
                     [b_store[5, tr] - m_perp * a_store(5, tr)]
                     [b_store[4, tr] - m_perp * a_store(4, tr)]
                     [b_store[3, tr] - m_perp * a_store(3, tr)]
                     [b_store[2, tr] - m_perp * a_store(2, tr)]
                     [b_store[1, tr] - m_perp * a_store(1, tr) + m_perp * xq[index + 1] - yq[index + 1]]]
                time = np.roots(p)
                for j in range(0, len(time)):
                    if ~any(np.imag(time[j])) == 1:
                        if time[j] > 0:
                            T_fin = time[j]

            for ii in range(0, num_foc_t):
                xd_n[ii][0] = [1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5] * a_store[:, tr]
                yd_n[ii][0] = [1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5] * b_store[:, tr]

                td_n = td_n + (T_fin - T_i) / num_foc_t

            curv = curvature_fn(x_store, y_store)
            A = np.array([0, 1, 2 * T_fin, 3 * T_fin ** 2, 4 * T_fin ** 3, 5 * T_fin ** 4])
            B = np.dot(A, a_store)
            C = np.dot(A, b_store)
            vd_n = max(min(math.sqrt(B * B + C * C), V_max), 0)
            wd_n = vd_n * curv[num_foc_t + 1][0]
            D = np.array([0, 0, 2, 6 * T_fin, 12 * T_fin ^ 2, 20 * T_fin ^ 3])
            E = np.dot(D, a_store)
            F = np.dot(E, b_store)
            ad_n = min(math.sqrt(F * F + E * E), 1.2)

            for jj in range(0, num_foc_t):
                if xq[index + 1] - xq[index] < 0:
                    theta4[jj][0] = math.pi + math.atan((yd_n[jj + 1] - yd_n[jj]) / xd_n[jj + 1] - xd_n[jj])
                else:
                    math.atan((yd_n[jj + 1] - yd_n[jj]) / xd_n[jj + 1] - xd_n[jj])

            theta4_n = theta4[num_foc_t][0]

            x_store[:, tr] = xd_n
            y_store[:, tr] = yd_n
            iterate = iterate + 1

    return x_store, y_store, vd_n, ad_n, wd_n, theta4_n, index


def initialization1(velocity, acceleration_in_x, z, time, p, indexes, angle):
    global xq, yq
    traj = 1

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

    return vx_i, vx_f, vy_i, vy_f, ax_i, ax_f, ay_i, ay_f, t_i, t_f, v_f, a_f, set_x, set_y, P_matrix, traj


ogm = environment_load()
xq = ogm[0, 4, :]
yq = ogm[1, 4, :]

x_set_init = 0
y_set_init = 2

T = [], V = []
V_init = 0
t_init = 0
A_init = 1
T[0] = t_init
V[0] = V_init

id = 23
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

follow_dis = critical_dis
traj = 1
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
                 traj) = initialization1(vel, a_long, w, t, i, index, theta)

            # elif state == 2 and phase == 2:
            #     [Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P,
            #      traj] = initialization_3_5_traj(vel, a_long, w, t, i, index, theta)
            #
            # elif state ==2 and phase ==1:
            #     [Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, traj, itr2,
            #      m_perp3] = initialization_2(vel, a_long, w, i, dist, itr2, vd_n, wd_n, ad_n, index, theta, theta4)
            #     xNormalLine1 = (1 / m_perp3) * (yq - dyn1_y) + dyn1_x

            for h in range(1, traj):
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
                    y_cen[f, 1] = yq(index, 1) + f * ((yq[index + 1, 0] - yq[index, 0]) / num_foc_t)

                x_store[:, h] = xd
                y_store[:, h] = yd

                a_coefficient_store[:, h] = a_coeff
                b_coefficient_store[:, h] = b_coeff
                time_store[:, h] = TD
