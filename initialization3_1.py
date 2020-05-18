import numpy as np
import math

xq = []
yq = []
V_max = 5
curvature = []
W_f = []
x_set = []
y_set = []
mtang4 = 0



def curvature_fn(x, y):
    k = np.zeros(len(x), 1)
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)

    for j in range(0, len(x)):
        k[j, 0] = (dx[j, 0] * ddy[j, 0] - dy[j, 0] * ddx[j, 0]) / (dx[j, 0] * dx[j, 0] + dy[j, 0] * dy[j, 0])

    return k


def initialization3_1(x_init, y_init, vel, vd_n, wd_n, ad_n, index, theta4):
    global W_f, x_set, y_set, mtang4
    num_foc_t = 10
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

        V_f = min(vel[:, 0])

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

        g1 = np.array([[x_init, Vx_i, Ax_i, xt_f, Vx_f, Ax_f]]).T
        g2 = np.array([[y_init, Vy_i, Ay_i, yt_f, Vy_f, Ay_f]]).T

        a_coeff = np.linalg.lstsq(P, g1)
        b_coeff = np.linalg.lstsq(P, g2)

        xd = np.zeros((num_foc_t + 1, 1))
        yd = np.zeros((num_foc_t + 1, 1))
        td = T_i

        for f in range(0, num_foc_t + 1):
            xd[f, 1] = [1, td, td ** 2, td ** 3, td ** 4, td ** 5] * a_coeff
            yd[f, 1] = [1, td, td ** 2, td ** 3, td ** 4, td ** 5] * b_coeff
            td = td + (T_f - T_i) / num_foc_t

        curvature[:, 0] = curvature_fn(xd, yd)

        if max(curvature[:, 0] <= 0.15):
            x_set = xq[index + 1]
            y_set = yq[index + 1]
            iteration = iteration + 1
        else:
            index = index + 1
            j = j + 1

        return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory, mtang4
