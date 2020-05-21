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


def menger_curv(x, y):
    k_matrix = np.zeros((len(x), 1))
    for qq in range(1, len(x) - 1):
        area1 = x[qq - 1] * (y[qq] - y[qq + 1])
        area2 = x[qq] * (y[qq + 1] - y[qq - 1])
        area3 = x[qq + 1] * (y[qq - 1] - y[qq])
        area = 0.5 * (area1 + area2 + area3)
        s1 = math.sqrt((x[qq - 1] - x[qq]) ** 2 + (y[qq - 1] - y[qq]) ** 2)
        s2 = math.sqrt((x[qq + 1] - x[qq]) ** 2 + (y[qq + 1] - y[qq]) ** 2)
        s3 = math.sqrt((x[qq - 1] - x[qq + 1]) ** 2 + (y[qq - 1] - y[qq + 1]) ** 2)
        s4 = s1 * s2 * s3
        result = area / s4
        k_matrix[qq] = 4 * result
    k_matrix[0] = k_matrix[1]
    k_matrix[len(x) - 1] = k_matrix[len(x) - 2]
    return k_matrix


def vel_prof(x, y, v0, a0, t0, current_state):
    global v_obs, V_max, critical_dis, follow_dis, dyn_x, dyn_y, dyn1_x, dyn1_y
    length_of_array_x = len(x)
    vel_matrix = np.zeros((length_of_array_x, 1))
    longitudinal_acceleration = np.zeros((length_of_array_x, 1))
    wt = np.zeros((length_of_array_x, 1))
    current_time = np.zeros((length_of_array_x, 1))
    c = menger_curv(x, y)
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

    for o in range(1, length_of_array_x):
        if c[o] != 0:  # turning is required
            v_all = min(v_max, math.sqrt(a_lat_max / abs(c[o])))
            temp = math.sqrt(
                vel_matrix[o - 1] ** 2 + 2 * acc_long * math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2))
            vel_matrix[o] = max(min(temp, v_all), 0)  # ensure non-negative value of velocity

            temp1 = (vel_matrix[o] ** 2 - vel_matrix[o - 1] ** 2) / (
                    2 * math.sqrt((x[o - 1] - x[o]) ** 2 + (y[o - 1] - y[o]) ** 2))
            longitudinal_acceleration[o] = min(temp1, acc_long)

            wt[0] = np.dot(v0, c[o])
            wt[o] = np.dot(vel_matrix[o], c[o])

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


def overtake_1(a_store, b_store, store_x, store_y, T_initial, kk, tang_m, perpendicular_m, index_number):
    global T_fin, theta4_n, ad_n, vd_n, wd_n
    trajectory1 = 5
    num_t = 10
    td_n = T_initial

    for tr in range(0, trajectory1):
        iterate = 0
        while iterate == 0 and tr == kk:
            if tang_m == 0:
                p = [[a_store[5, tr]],
                     [a_store[4, tr]],
                     [a_store[3, tr]],
                     [a_store[2, tr]],
                     [a_store[1, tr]],
                     [a_store[0, tr] - xq[index_number + 1]]]
                time = np.roots(p)
                for mm in range(0, len(time)):
                    if ~any(np.imag(time[mm])) == 1:
                        if time[mm] > 0:
                            T_fin = time[mm]

            else:
                p = [[b_store[5, tr] - perpendicular_m * a_store[5, tr]],
                     [b_store[4, tr] - perpendicular_m * a_store[4, tr]],
                     [b_store[3, tr] - perpendicular_m * a_store[3, tr]],
                     [b_store[2, tr] - perpendicular_m * a_store[2, tr]],
                     [b_store[1, tr] - perpendicular_m * a_store[1, tr]],
                     [b_store[0, tr] - perpendicular_m * a_store[0, tr] + perpendicular_m * xq[index_number + 1] - yq[
                         index_number + 1]]]
                time = np.roots(p)
                for mm in range(0, len(time)):
                    if ~any(np.imag(time[mm])) == 1:
                        if time[mm] > 0:
                            T_fin = time[mm]

            for ii in range(0, num_t + 1):
                xd_n[ii] = np.dot([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5], a_store[:, tr])
                yd_n[ii] = np.dot([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5], b_store[:, tr])
                td_n = td_n + (T_fin - T_initial) / num_t

            curv = curvature_fn(xd_n, yd_n)
            A = np.array([0, 1, 2 * T_fin, 3 * T_fin ** 2, 4 * T_fin ** 3, 5 * T_fin ** 4])
            B = np.dot(A, a_store)
            C = np.dot(A, b_store)
            vd_n = max(min(math.sqrt(B * B + C * C), V_max), 0)
            wd_n = vd_n * curv[num_t + 1][0]
            D = np.array([0, 0, 2, 6 * T_fin, 12 * T_fin ^ 2, 20 * T_fin ^ 3])
            E = np.dot(D, a_store)
            F = np.dot(E, b_store)
            ad_n = min(math.sqrt(F * F + E * E), 1.2)

            for mm in range(0, num_t):
                if xq[index_number + 1] - xq[index_number] < 0:
                    theta4[mm, 0] = math.pi + math.atan((yd_n[mm + 1] - yd_n[mm]) / xd_n[mm + 1] - xd_n[mm])
                else:
                    theta4[mm, 0] = math.atan((yd_n[mm + 1] - yd_n[mm]) / xd_n[mm + 1] - xd_n[mm])

            theta4_n = theta4[num_t, 0]

            store_x[:, tr] = xd_n
            store_y[:, tr] = yd_n
            iterate = iterate + 1

    return store_x, store_y, vd_n, ad_n, wd_n, theta4_n, T_fin


def overtake_1_2(a_store, b_store, store_x, store_y, T_initial, tang_m, perpendicular_m, index_no):
    global T_fin, theta4_n, ad_n, vd_n, wd_n
    trajectory2 = 1
    num_t = 10
    td_n = T_initial

    for tr in range(0, trajectory2):
        iterate = 0
        while iterate == 0:
            if tang_m == 0:
                p = [[a_store[5, tr]]
                     [a_store[4, tr]]
                     [a_store[3, tr]]
                     [a_store[2, tr]]
                     [a_store[1, tr]]
                     [a_store[0, tr] - xq[index_no + 1]]]
                time = np.roots(p)
                for nn in range(0, len(time)):
                    if ~any(np.imag(time[nn])) == 1:
                        if time[nn] > 0:
                            T_fin = time[nn]

            else:
                p = [[b_store[5, tr] - perpendicular_m * a_store[5, tr]],
                     [b_store[4, tr] - perpendicular_m * a_store[4, tr]],
                     [b_store[3, tr] - perpendicular_m * a_store[3, tr]],
                     [b_store[2, tr] - perpendicular_m * a_store[2, tr]],
                     [b_store[1, tr] - perpendicular_m * a_store[1, tr]],
                     [b_store[0, tr] - perpendicular_m * a_store[0, tr] + perpendicular_m * xq[index_no + 1] - yq[
                         index_no + 1]]]
                time = np.roots(p)
                for nn in range(0, len(time)):
                    if ~any(np.imag(time[nn])) == 1:
                        if time[nn] > 0:
                            T_fin = time[nn]

            for ii in range(0, num_t + 1):
                xd_n[ii, 0] = np.dot([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5], a_store[:, tr])
                yd_n[ii, 0] = np.dot([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5], b_store[:, tr])
                td_n = td_n + (T_fin - T_initial) / num_t

            curv = curvature_fn(xd_n, yd_n)
            A = np.array([0, 1, 2 * T_fin, 3 * T_fin ** 2, 4 * T_fin ** 3, 5 * T_fin ** 4])
            B = np.dot(A, a_store)
            C = np.dot(A, b_store)
            vd_n = max(min(math.sqrt(B * B + C * C), V_max), 0)
            wd_n = vd_n * curv[num_t + 1][0]
            D = np.array([0, 0, 2, 6 * T_fin, 12 * T_fin ^ 2, 20 * T_fin ^ 3])
            E = np.dot(D, a_store)
            F = np.dot(E, b_store)
            ad_n = min(math.sqrt(F * F + E * E), 1.2)

            for nn in range(0, num_t):
                if xq[index_no + 1] - xq[index_no] < 0:
                    theta4[nn, 0] = math.pi + math.atan((yd_n[nn + 1] - yd_n[nn]) / xd_n[nn + 1] - xd_n[nn])
                else:
                    theta4[nn, 0] = math.atan((yd_n[nn + 1] - yd_n[nn]) / xd_n[nn + 1] - xd_n[nn])

            theta4_n = theta4[num_t, 0]

            store_x[:, tr] = xd_n
            store_y[:, tr] = yd_n
            iterate = iterate + 1

    return store_x, store_y, vd_n, ad_n, wd_n, theta4_n, T_fin


def initialization1(velocity, acceleration_in_x, omega, time, p, indexes, angle):
    global xq, yq, trajectory
    trajectory = 1

    v_i = velocity[p]
    v_f = velocity[p + 1]
    t_i = 0
    t_f = time[p + 1] - time[p]
    w_i = omega[p]
    w_f = omega[p + 1]
    a_i = acceleration_in_x[p]
    a_f = acceleration_in_x[p + 1]

    m_tang2 = (yq[indexes + 2] - yq[indexes + 1]) / (xq[indexes + 2] - xq[indexes + 1])
    if xq[indexes + 1] - xq[indexes] < 0:
        theta2 = math.pi + math.atan(m_tang2)
    else:
        theta2 = math.atan(m_tang2)

    P_matrix = np.array([[1, t_i, t_i ** 2, t_i ** 3, t_i ** 4, t_i ** 5],
                         [0, 1, 2 * t_i, 3 * (t_i ** 2), 4 * (t_i ** 3), 5 * (t_i ** 4)],
                         [0, 0, 2, 6 * t_i, 12 * (t_i ** 2), 20 * (t_i ** 3)],
                         [1, t_f, t_f ** 2, t_f ** 3, t_f ** 4, t_f ** 5],
                         [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
                         [0, 0, 2, 6 * t_f, 12 * (t_f ** 2), 20 * (t_f ** 3)]])

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


def initialization3_1(x_init, y_init, vel, vd_n, wd_n, ad_n, index, theta4):
    global W_f, x_set, y_set, mtang4
    num_t = 10
    trajectory = 1
    T_i = 0
    j = 0
    iteration = 0

    while iteration == 0:
        xf = np.array([[xq[index], xq[index + 1], xq[index + 2]]]).T
        yf = np.array([[yq[index], yq[index + 1], yq[index + 2]]]).T
        dist = math.sqrt(((x_init - xq[index + 1]) ** 2) + (y_init - yq[index + 1]) ** 2)

        meng = curvature_fn(xf, yf)
        if j > 0:
            mtang4 = (yq[index + 1] - yq[index]) / (xq[index + 1] - xq[index])
        elif j == 0:
            mtang4 = (yq[index + 2] - yq[index + 1]) / (xq[index + 2] - xq[index + 1])

        if xq[index + 2] - xq[index + 1] < 0:
            theta_4 = math.pi + math.atan(mtang4)
        else:
            theta_4 = math.atan(mtang4)

        V_f = min(vel[:])

        if j > 0:
            W_f = V_f * meng[0, 0]
        elif j == 0:
            W_f = V_f * meng[1, 0]

        A_f = 0
        T_f = dist / vd_n
        P = [[1, T_i, T_i ** 2, T_i ** 3, T_i ** 4, T_i ** 5],
             [0, 1, 2 * T_i, 3 * (T_i ** 2), 4 * (T_i ** 3), 5 * (T_i ** 4)],
             [0, 0, 2, 6 * T_i, 12 * (T_i ** 2), 20 * (T_i ** 3)],
             [1, T_f, T_f ** 2, T_f ** 3, T_f ** 4, T_f ** 5],
             [0, 1, 2 * T_f, 3 * (T_f ** 2), 4 * (T_f ** 3), 5 * (T_f ** 4)],
             [0, 0, 2, 6 * T_f, 12 * (T_f ** 2), 20 * (T_f ** 3)]]

        Vx_i = vd_n * math.cos(theta4)
        Vx_f = V_f * math.cos(theta_4)
        Vy_i = vd_n * math.sin(theta4)
        Vy_f = V_f * math.sin(theta_4)
        Ax_i = ad_n * math.cos(theta4) - vd_n * math.sin(theta4) * wd_n
        Ax_f = A_f * math.cos(theta_4) - V_f * math.sin(theta_4) * W_f
        Ay_i = ad_n * math.sin(theta4) + vd_n * math.cos(theta4) * wd_n
        Ay_f = A_f * math.sin(theta_4) + V_f * math.cos(theta_4) * W_f

        xt_f = xq[index + 1]
        yt_f = yq[index + 1]

        g_matrix1 = np.array([[x_init, Vx_i, Ax_i, xt_f, Vx_f, Ax_f]]).T
        g_matrix2 = np.array([[y_init, Vy_i, Ay_i, yt_f, Vy_f, Ay_f]]).T

        a_coeff = np.linalg.lstsq(P, g_matrix1)
        b_coeff = np.linalg.lstsq(P, g_matrix2)

        xd = np.zeros((num_t + 1, 1))
        yd = np.zeros((num_t + 1, 1))
        td = T_i

        for f in range(0, num_t + 1):
            xd[f, 1] = [1, td, td ** 2, td ** 3, td ** 4, td ** 5] * a_coeff
            yd[f, 1] = [1, td, td ** 2, td ** 3, td ** 4, td ** 5] * b_coeff
            td = td + (T_f - T_i) / num_t

        curvature = menger_curv(xd, yd)

        if max(curvature[:] <= 0.15):
            x_set = xq[index + 1]
            y_set = yq[index + 1]
            iteration = iteration + 1
        else:
            index = index + 1
            j = j + 1

        return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory, mtang4


def initialization_2(vel, a_long, w, i, dist, itr2, vd_n, wd_n, ad_n, index, theta, theta4, itr4, jj):
    global m_tang3, xf, yf, x_f, y_f
    l_off = 2
    trajectory = 5
    if itr4 == 1:
        jj = jj + 1

    z = np.subtract(xq, dyn1_x)
    ee = np.argmin(np.abs(z))
    if itr4 == 1:
        cc = ee + jj
        itr4 = 0
    else:
        cc = ee

    if xq[cc] > dyn1_x and yq[cc] == dyn1_y:
        m_tang3 = (yq[cc] - dyn1_y) / (xq[cc] - dyn1_x)
    elif xq[cc] <= dyn1_x and yq[cc] == dyn1_y:
        m_tang3 = (yq[cc] - dyn1_y) / (xq[cc] - dyn1_x)
    elif yq[cc] != dyn1_y:
        m_tang3 = yq[cc + 1] - yq[cc] / (xq[cc + 1] - xq[cc])

    m_perp3 = -1 / m_tang3
    if xq[index + 1] - xq[index] < 0:
        theta3 = math.pi + math.atan(m_tang3)
    else:
        theta3 = math.atan(m_tang3)

    itr2 = itr2 + 1

    if xq[cc] < dyn1_x and yq[cc] == dyn1_y:
        xf = np.array([[xq[cc], dyn1_x, xq[cc + 1]]]).T
        yf = np.array([[yq[cc], dyn1_y, yq[cc + 1]]]).T
        x_f = xq[cc + 1]
        y_f = yq[cc + 1]
    elif xq[cc] > dyn1_x and yq[cc] == dyn1_y:
        xf = np.array([[xq[cc - 1], dyn1_x, xq[cc]]]).T
        yf = np.array([[yq[cc - 1], dyn1_y, yq[cc]]]).T
        x_f = xq[cc]
        y_f = yq[cc]
    elif xq[cc] == dyn1_x and yq[cc] == dyn1_y:
        xf = np.array([[xq[cc - 1], dyn1_x, xq[cc + 1]]]).T
        yf = np.array([[yq[cc - 1], dyn1_y, yq[cc + 1]]]).T
        x_f = xq[cc + 1]
        y_f = yq[cc + 1]
    elif yq[cc] != dyn1_y:
        xf = np.array([[xq[cc - 1], xq[cc], xq[cc + 1]]]).T
        yf = np.array([[yq[cc - 1], yq[cc], yq[cc + 1]]]).T
        x_f = xq[cc + 1]
        y_f = yq[cc + 1]

    if itr2 > 1:
        V_i = vd_n
    else:
        V_i = vel[i]

    V_f = V_max
    meng = curvature_fn(xf, yf)
    W_i = w[i]
    W_f = V_f * meng[1, 0]
    A_i = a_long[i]
    A_f = 0
    T_i = 0
    T_f = dist / V_i
    P = [[1, T_i, T_i ** 2, T_i ** 3, T_i ** 4, T_i ** 5],
         [0, 1, 2 * T_i, 3 * (T_i ** 2), 4 * (T_i ** 3), 5 * (T_i ** 4)],
         [0, 0, 2, 6 * T_i, 12 * (T_i ** 2), 20 * (T_i ** 3)],
         [1, T_f, T_f ** 2, T_f ** 3, T_f ** 4, T_f ** 5],
         [0, 1, 2 * T_f, 3 * (T_f ** 2), 4 * (T_f ** 3), 5 * (T_f ** 4)],
         [0, 0, 2, 6 * T_f, 12 * (T_f ** 2), 20 * (T_f ** 3)]]

    if itr2 == 1:
        Vx_i = V_i * math.cos(theta3)
        Vx_f = V_f * math.cos(theta3)
        Vy_i = V_i * math.sin(theta3)
        Vy_f = V_f * math.sin(theta3)
        Ax_i = A_i * math.cos(theta) - V_i * math.sin(theta) * W_i
        Ax_f = A_f * math.cos(theta3) - V_f * math.sin(theta3) * W_f
        Ay_i = A_i * math.sin(theta) + V_i * math.cos(theta) * W_i
        Ay_f = A_f * math.sin(theta3) + V_f * math.cos(theta3) * W_f

    else:
        Vx_i = vd_n * math.cos(theta4)
        Vy_i = vd_n * math.sin(theta4)
        Ax_i = ad_n * math.cos(theta4) - vd_n * math.sin(theta4) * wd_n
        Ay_i = ad_n * math.sin(theta4) + vd_n * math.cos(theta4) * wd_n
        Vx_f = V_f * math.cos(theta3)
        Vy_f = V_f * math.sin(theta3)
        Ax_f = A_f * math.cos(theta3) - V_f * math.sin(theta3) * W_f
        Ay_f = A_f * math.sin(theta3) + V_f * math.cos(theta3) * W_f

    x_set = np.array([[x_f - math.sin(theta3) * l_off, x_f - 0.5 * math.sin(theta3) * l_off, x_f,
                       x_f + math.sin(theta3) * l_off, x_f + 1.5 * math.sin(theta3) * l_off]]).T
    y_set = np.array([[y_f + math.cos(theta3) * l_off, y_f + 0.5 * math.cos(theta3) * l_off, y_f,
                       y_f - math.cos(theta3) * l_off, y_f - 1.5 * math.cos(theta3) * l_off]]).T

    return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory, itr2, m_perp3, itr4, jj


def cost(xd1, yd1, x_centre, y_centre, phase_chosen):
    global total_cost
    if max(menger_curv[:, 0]) <= 0.15:
        if phase_chosen == 2:
            cost1 = 0
            cost2 = np.sum(np.sqrt(np.add(np.square(x_centre - xd1), np.square(y_centre - yd1))))
            total_cost = 2 * cost1 + 0.5 * cost2
        else:
            cost1 = max(min(np.sqrt(np.add(np.square(xd1 - dyn1_x), np.square(yd1 - dyn1_y))) - 2), 0.01)
            cost2 = np.sum(np.sqrt(np.add(np.square(x_centre - xd1), np.square(y_centre - yd1))))
            total_cost = 2 * cost1 + 0.5 * cost2

    else:
        total_cost = math.inf

    return total_cost


ogm = environment_load()
xq = ogm[0, 3, :]
yq = ogm[1, 3, :]
x_set_init = 0
y_set_init = 2
V_init = 0
t_init = 0
A_init = 1
T = t_init
V = V_init

# id = 23
id2 = 41

lane_status = 0

# variable definition

# T_fin = []
xd_n = np.zeros((11, 1))
yd_n = np.zeros((11, 1))
# theta4_n = []
# g1 = []
# g2 = []
# time_store = []
# cost_store = []
# curvature = []
# xf = []
# yf = []
# x_f = []
# y_f = []

num_foc_t = 10
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
A_f = 0
follow_dis = critical_dis
trajectory = 1
itr2 = 0
itr1 = 0
vd_n = 0
wd_n = 0
ad_n = 0
theta4 = 0
a = 0
itr4 = 0
jj = 0
cost_store = np.zeros((1, 5))
# m_tang3 = 0
# m_tang4 = 0

while index < len(xq):
    global x_set, y_set, Ax_f, Ax_i, Ay_f, Ay_i, Vx_f, Vx_i, Vy_f, Vy_i, P, T_i, T_f, V_f, xNormalLine1, xNormalLine, m_tang, T_add, var, a_coefficient_store, b_coefficient_store, x_store, y_store, k, time_store
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
        m_tang_np = (yq[index + 2] - yq[index + 1]) / (xq[index + 2] - xq[index + 1])
        m_perp_np = -1 / m_tang_np

        xNormalLine = np.dot((1 / m_perp), np.add((yq - yq[index + 1]), xq[index]))
        if xq[index + 1] - xq[index] < 0:
            theta = math.pi + math.atan(m_tang)
        else:
            theta = math.atan(m_tang)

        # phase1 corresponds to overtaking and phase2 corresponds to returning to reference line

        if state != 2 and phase == 0:
            (Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P,
             trajectory) = initialization1(vel, a_long, w, t, i, index, theta)

        # elif state == 2 and phase == 2:
        #     (Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P,
        #      trajectory) = initialization3_1(x_set_init, y_set_init, vel, vd_n, wd_n, ad_n, index, theta4)
        # elif state == 2 and phase == 1:
        #     [Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory, itr2,
        #      m_perp3, itr4, jj] = initialization_2(vel, a_long, w, i, dist, itr2, vd_n, wd_n, ad_n, index, theta,
        #                                            theta4, itr4, jj)
        #     xNormalLine1 = (1 / m_perp3) * (yq - dyn1_y) + dyn1_x

        for h in range(0, trajectory):
            xt_f = x_set[h]
            yt_f = y_set[h]

            if state == 2 and phase == 2:
                # inspect here
                g1 = np.zeros(6, 1)
                g2 = np.zeros(6, 1)
                g1[h] = np.array([[x_set_init], [Vx_i], [Ax_i], [xt_f], [Vx_f[h]], [Ax_f[h]]])
                g2[h] = np.array([[y_set_init], [Vy_i], [Ay_i], [yt_f], [Vy_f[h]], [Ay_f[h]]])

                a_coeff = np.linalg.lstsq(P[:, :, h], g1[:, h])
                b_coeff = np.linalg.lstsq(P[:, :, h], g2[:, h])

            else:
                g1 = np.array([[x_set_init], [Vx_i], [Ax_i], [xt_f], [Vx_f], [Ax_f]])
                g2 = np.array([[y_set_init], [Vy_i], [Ay_i], [yt_f], [Vy_f], [Ay_f]])

                a_coeff = np.linalg.lstsq[P, g1]
                b_coeff = np.linalg.lstsq[P, g2]

            num_foc_t = 10
            xd = np.zeros((num_foc_t + 1, 1))
            yd = np.zeros((num_foc_t + 1, 1))
            xd_o = np.zeros((num_foc_t + 1, 1))
            yd_o = np.zeros((num_foc_t + 1, 1))
            x_cen = np.zeros((num_foc_t + 1, 1))
            y_cen = np.zeros((num_foc_t + 1, 1))
            TD = np.zeros((num_foc_t + 1, 1))
            vd = np.zeros((num_foc_t + 1, 1))
            wd = np.zeros((num_foc_t + 1, 1))
            td = T_i
            x_point = np.zeros((num_foc_t + 1, 1))
            y_point = np.zeros((num_foc_t + 1, 1))
            x_pnt = np.zeros((num_foc_t + 1, 1))
            y_pnt = np.zeros((num_foc_t + 1, 1))

            for f in range(0, num_foc_t + 1):
                xd[f] = [1, td, td ** 2, td ** 3, td ** 4, td ** 5] * a_coeff
                yd[f] = [1, td, td ** 2, td ** 3, td ** 4, td ** 5] * b_coeff

                td_old = td
                if state == 2 and phase == 2:
                    td = td + (T_f[h] - T_i) / num_foc_t
                else:
                    td = td + (T_f - T_i) / num_foc_t

                TD[f] = td

                x_cen[f] = np.add(xq[index], np.dot(f, np.divide(np.subtract(xq[index + 1], xq[index]), num_foc_t)))
                y_cen[f] = np.add(yq[index], np.dot(f, np.divide(np.subtract(yq[index + 1], yq[index]), num_foc_t)))

            if trajectory == 1:
                x_store = xd
                y_store = yd
                x_store_n = x_store
                y_store_n = y_store
                a_coefficient_store = a_coeff
                b_coefficient_store = b_coeff
                time_store = TD

            elif trajectory == 5:
                x_store = np.zeros(11, 1)
                y_store = np.zeros(11, 1)
                x_store_n = np.zeros(11, 1)
                y_store_n = np.zeros(11, 1)
                a_coefficient_store = np.zeros(11, 1)
                b_coefficient_store = np.zeros(11, 1)
                time_store = np.zeros(11, 1)

                x_store[h] = xd
                y_store[h] = yd
                x_store_n[h] = x_store[h]
                y_store_n[h] = y_store[h]
                a_coefficient_store[h] = a_coeff
                b_coefficient_store[h] = b_coeff
                time_store[h] = TD

            d = np.argmin(np.abs(xq - dyn1_x))
            if m_tang == math.inf:
                if yq[d] < dyn1_y:
                    var = d + 1
                elif yq[d] >= dyn1_y:
                    var = d
            else:
                if xq[d] < dyn1_x:
                    var = d + 1
                elif xq[d] >= dyn1_x:
                    var = d

            for q in range(1, num_foc_t + 1):
                if index < var:  # checking for forward distance & itr1 flag is phase=2 detection(0 means phase~=2 and >0 means phase=2)
                    if math.sqrt((xd[q] - dyn1_x) ** 2 + (yd[q] - dyn1_y) ** 2) < 11:
                        if lane_status == 0:
                            phase = 1
                            state = 2
                            x_pnt[q] = xd[q]
                            y_pnt[q] = yd[q]
                            s = np.where(x_pnt)
                            itr2 = itr2 + 1  # itr2 is the flag just to check immediate detection of obstacle
                            if itr2 == 1:  # when detected
                                x_store[1:s, h] = xd[1:s, 1]  # not done yet
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

                        break
                    elif phase == 0:
                        state = 0

            if itr2 > 1 or phase == 1:
                cost_store[h] = total_cost
                break

        if itr2 > 1 or phase == 1:
            if cost_store[:] == math.inf:
                itr4 = 1
                break
            else:
                k = np.argmin(cost_store)
                jj = 0
            [x_store, y_store, vd_n, ad_n, wd_n, theta4, T_add] = overtake_1(a_coefficient_store,
                                                                             b_coefficient_store, x_store,
                                                                             y_store, T_i, k, m_tang, m_perp, index)

        elif phase == 2:
            [x_store, y_store, vd_n, ad_n, wd_n, theta4, T_add] = overtake_1_2(a_coefficient_store,
                                                                               b_coefficient_store, x_store,
                                                                               y_store, T_i, m_tang, m_perp, index)

        if state == 2 and itr2 > 1:
            if phase == 1:
                x_set_init = x_store[k]
                y_set_init = y_store[k]
                dist = math.sqrt((x_set_init - dyn1_x) ** 2 + (y_set_init - dyn1_y) ** 2)
            elif phase == 2:
                x_set_init = x_store[0]
                y_set_init = y_store[0]
                dist = math.sqrt((x_set_init - dyn1_x) ** 2 + (y_set_init - dyn1_y) ** 2)
                itr1 = itr1 + 1

        else:
            x_set_init = x_store[0]
            y_set_init = y_store[0]
            dist = math.sqrt((x_set_init - dyn1_x) ** 2 + (y_set_init - dyn1_y) ** 2)

    # plots

    plt.plot(ogm[0, 2, :], ogm[1, 2, :], linewidth=2, c='black')
    plt.plot(ogm[0, 4, :], ogm[1, 4, :], c='cyan')
    plt.plot(ogm[0, 6, :], ogm[1, 6, :], linestyle='--')
    plt.plot(ogm[0, 10, :], ogm[1, 10, :], linewidth=2, c='black')
    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')
    plt.axis([0, 165, -35, 35])
    plt.show()

    for b in range(0, trajectory):
        plt.plot(x_store[:, b], y_store[:, b], linestyle='-', linewidth=1, c='blue')
        plt.plot(xNormalLine, yq)
        plt.show()

    if state == 2 and itr2 > 1:
        plt.plot(xNormalLine1, yq)
        if phase == 1:
            plt.plot(x_store[:, k], y_store[:, k], linestyle='-', linewidth=2, c='black')
            for r in range(0, num_foc_t + 1):
                plt.plot(x_store[r, k], y_store[r, k], markersize=6)
                plt.plot(dyn1_x, dyn1_y, markersize=10, markerfacecolor='magenta')
                plt.title('Detected state = ', str(state))
                plt.show()
                plt.pause(0.001)

        elif phase == 2:
            if itr == 1:
                plt.plot(x_store[:, k], y_store[:, k], linestyle='-', linewidth=2, c='blue')
                for r in range(0, num_foc_t + 1):
                    plt.plot(x_store[r, k], y_store[r, k], markersize=6)
                    plt.plot(dyn1_x, dyn1_y, markerSize=10, markerFaceColor='magenta')
                    plt.title(['Detected state = ', str(state)])
                    plt.show()
                    plt.pause(0.01)
            elif itr1 > 1:
                plt.plot(x_store[:, 0], y_store[:, 0], linestyle='-', linewidth=2, c='blue')
                for r in range(0, num_foc_t + 1):
                    plt.plot(x_store[r, 0], y_store[r, 10], markersize=6)
                    plt.plot(dyn1_x, dyn1_y, markerSize=6, markerFaceColor='magenta')
                    plt.title(['Detected state = ', str(state)])
                    plt.show()
                    plt.pause(0.01)

        else:
            plt.plot(x_store[:, 0], y_store[:, 0], linestyle='-', linewidth='2', color='red')
            for r in range(0, num_foc_t + 1):
                plt.plot(x_store(r, 1), y_store(r, 1), markersize=6)
                plt.plot(dyn1_x, dyn1_y, markerSize=10, markerFaceColor='magenta')
                plt.title(['Detected state = ', str(state)])
                plt.show()
                plt.pause(0.01)

    if state == 2 and itr2 > 1:
        if phase == 1:
            if m_tang == 0:
                if x_store[num_foc_t + 1, k] > dyn1_x:
                    phase = 2
                    itr1 = itr1 + 1
            elif m_tang == math.inf:
                if y_store[num_foc_t + 1, k] > dyn1_y:
                    phase = 2
                    itr1 = itr1 + 1
            elif m_tang != 0:
                if xq[index + 1] - xq[index] < 0 or yq[index + 1] - yq[index] < 0:
                    if x_store[num_foc_t + 1, k] <= dyn1_x and y_store[num_foc_t + 1, k] <= dyn1_y:
                        phase = 2
                        itr1 = itr1 + 1
                else:
                    if x_store[num_foc_t + 1, k] > dyn1_x:
                        phase = 2
                        itr1 = itr1 + 1

        elif phase == 2:
            if x_store[num_foc_t + 1, 1] == xq[index + 1] and y_store[num_foc_t + 1] == yq[index + 1]:
                phase = 0
                state = 0
                itr = itr + 1

    if itr2 == 0:
        t_init = t[i + 1]
    elif itr2 == 1:
        t_init = t[i] + time_store[i]
    elif itr2 > 1 and state == 2:
        t_init = t[i] + T_add

    i = i + 1

    if state != prev_state:
        if phase == 0 and itr != 0:
            V_init = vd_n
            A_init = ad_n
            index = index + 1
            itr1 = 0

        itr2 = 0
        break
    if i >= horizon:
        state = 5

    index = index + 1

    if phase == 2:
        V_init = vd_n
        A_init = ad_n
    else:
        V_init = V_f
        A_init = A_f
