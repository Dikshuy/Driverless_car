import numpy as np
import math

xq = []
yq = []
V_max = 0
T_fin = []
theta4 = []
x_d = []
y_d = []
theta4_n = []
ad_n = []
vd_n = []
wd_n = []


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
                p = [[a_store[5, tr]]
                     [a_store[4, tr]]
                     [a_store[3, tr]]
                     [a_store[2, tr]]
                     [a_store[1, tr]]
                     [a_store[0, tr] - xq[index + 1]]]
                time = np.roots(p)
                for j in range(0, len(time)):
                    if ~any(np.imag(time[j])) == 1:
                        if time[j] > 0:
                            T_fin = time[j]

            else:
                p = [[b_store[5, tr] - m_perp * a_store[6, tr]],
                     [b_store[4, tr] - m_perp * a_store(5, tr)]
                     [b_store[3, tr] - m_perp * a_store(4, tr)]
                     [b_store[2, tr] - m_perp * a_store(3, tr)]
                     [b_store[1, tr] - m_perp * a_store(2, tr)]
                     [b_store[0, tr] - m_perp * a_store(1, tr) + m_perp * xq[index + 1] - yq[index + 1]]]
                time = np.roots(p)
                for j in range(0, len(time)):
                    if ~any(np.imag(time[j])) == 1:
                        if time[j] > 0:
                            T_fin = time[j]

            for ii in range(0, num_foc_t):
                x_d[ii][0] = [1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5] * a_store[:, tr]
                y_d[ii][0] = [1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5] * b_store[:, tr]

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
                    theta4[jj][0] = math.pi + math.atan((y_d[jj + 1] - y_d[jj]) / x_d[jj + 1] - x_d[jj])
                else:
                    math.atan((y_d[jj + 1] - y_d[jj]) / x_d[jj + 1] - x_d[jj])

            theta4_n = theta4[num_foc_t][0]

            x_store[:, tr] = x_d
            y_store[:, tr] = y_d
            iterate = iterate + 1

    return x_store, y_store, vd_n, ad_n, wd_n, theta4_n, index
