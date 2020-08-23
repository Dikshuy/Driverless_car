import numpy as np
import math

xq = []
yq = []
V_max = 0
T_fin = []
theta4 = []
xd_n = []
yd_n = []
theta4_n = []
ad_n = []
vd_n = []
wd_n = []
theta44 = []


def curvature_fn(x, y):
    k = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for j in range(0, len(x)):
        k[j, 0] = (dx[j, 0] * ddy[j, 0] - dy[j, 0] * ddx[j, 0]) / (dx[j, 0] * dx[j, 0] + dy[j, 0] * dy[j, 0])

    return k


def overtake_1(a_store, b_store, store_x, store_y, T_initial, kk, tang_m, perpendicular_m, index_number):
    global T_fin, theta4_n, ad_n, vd_n, wd_n, xq, yq
    trajectory1 = 5
    num_t = 10
    td_n = T_initial

    for tr in range(0, trajectory1):
        iterate = 0
        while iterate == 0 and tr == kk:
            if tang_m == 0:
                p = np.array([a_store[5, tr],
                              a_store[4, tr],
                              a_store[3, tr],
                              a_store[2, tr],
                              a_store[1, tr],
                              a_store[0, tr] - xq[index_number + 1]])

                times = np.roots(p)
                for mm in range(0, len(times)):
                    if times[mm].imag == 0:
                        if times[mm] > 0:
                            T_fin = times[mm].real

            else:
                p = np.array([b_store[5, tr] - perpendicular_m * a_store[5, tr],
                              b_store[4, tr] - perpendicular_m * a_store[4, tr],
                              b_store[3, tr] - perpendicular_m * a_store[3, tr],
                              b_store[2, tr] - perpendicular_m * a_store[2, tr],
                              b_store[1, tr] - perpendicular_m * a_store[1, tr],
                              b_store[0, tr] - perpendicular_m * a_store[0, tr] + perpendicular_m * xq[
                                  index_number + 1] - yq[
                                  index_number + 1]])

                times = np.roots(p)
                for mm in range(0, len(times)):
                    if times[mm].imag == 0:
                        if times[mm] > 0:
                            T_fin = times[mm].real

            for ii in range(0, num_t + 1):
                xd_n[ii] = np.dot(np.array([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5]),
                                  a_store[:, tr])
                yd_n[ii] = np.dot(np.array([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5]),
                                  b_store[:, tr])
                td_n = td_n + (T_fin - T_initial) / num_t

            curv = curvature_fn(xd_n, yd_n)
            A = np.array([0, 1, 2 * T_fin, 3 * T_fin ** 2, 4 * T_fin ** 3, 5 * T_fin ** 4])
            B = np.dot(A, a_store[:, tr])
            C = np.dot(A, b_store[:, tr])
            vd_n = max(min(math.sqrt(B * B + C * C), V_max), 0)
            wd_n = vd_n * curv[num_t]
            D = np.array([0, 0, 2, 6 * T_fin, 12 * (T_fin ** 2), 20 * (T_fin ** 3)], ndmin=2)
            E = np.dot(D, a_store[:, tr])
            F = np.dot(D, b_store[:, tr])
            ad_n = min(math.sqrt(F * F + E * E), 1.2)

            for mm in range(0, num_t):
                if xq[index_number + 1] - xq[index_number] < 0:
                    theta44[mm] = math.pi + math.atan((yd_n[mm + 1] - yd_n[mm]) / (xd_n[mm + 1] - xd_n[mm]))
                else:
                    theta44[mm] = math.atan((yd_n[mm + 1] - yd_n[mm]) / (xd_n[mm + 1] - xd_n[mm]))

            theta4_n = theta44[num_t - 1]
            store_x[:, tr] = xd_n
            store_y[:, tr] = yd_n
            iterate = iterate + 1
    print("Overtaking the obstacle")

    return store_x, store_y, vd_n, ad_n, wd_n, theta4_n, T_fin
