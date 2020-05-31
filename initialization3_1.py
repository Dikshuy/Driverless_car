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
theta4 = 0
Vx_i = 0
Vx_f = 0
Vy_i = 0
Vy_f = 0
Ax_i = 0
Ax_f = 0
Ay_i = 0
Ay_f = 0
V_f = 0
A_f = 0
P = []
T_f = 0


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


def initialization3_1(x_init, y_init, velocity, vel_n, omega_n, acc_n, index_no):
    global W_f, mtang4, Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, V_f, A_f, P, T_f, x_set, y_set, theta4
    num_t = 10
    trajectory = 1
    T_initial = 0
    j = 0
    iteration = 0

    while iteration == 0:
        xf1 = np.array([xq[index_no], xq[index_no + 1], xq[index_no + 2]])
        yf1 = np.array([yq[index_no], yq[index_no + 1], yq[index_no + 2]])
        distance = math.sqrt(((x_init - xq[index_no + 1]) ** 2) + (y_init - yq[index_no + 1]) ** 2)

        meng = curvature_fn(xf1, yf1)
        if j > 0:
            mtang4 = (yq[index_no + 1] - yq[index_no]) / (xq[index_no + 1] - xq[index_no])
        elif j == 0:
            mtang4 = (yq[index_no + 2] - yq[index_no + 1]) / (xq[index_no + 2] - xq[index_no + 1])

        if xq[index_no + 2] - xq[index_no + 1] < 0:
            theta_4 = math.pi + math.atan(mtang4)
        else:
            theta_4 = math.atan(mtang4)

        V_f = min(velocity[:])

        if j > 0:
            W_f = V_f * meng[0]
        elif j == 0:
            W_f = V_f * meng[1]

        A_f = 0
        T_f = distance / vel_n
        P = np.array([[1, T_initial, T_initial ** 2, T_initial ** 3, T_initial ** 4, T_initial ** 5],
                      [0, 1, 2 * T_initial, 3 * (T_initial ** 2), 4 * (T_initial ** 3), 5 * (T_initial ** 4)],
                      [0, 0, 2, 6 * T_initial, 12 * (T_initial ** 2), 20 * (T_initial ** 3)],
                      [1, T_f, T_f ** 2, T_f ** 3, T_f ** 4, T_f ** 5],
                      [0, 1, 2 * T_f, 3 * (T_f ** 2), 4 * (T_f ** 3), 5 * (T_f ** 4)],
                      [0, 0, 2, 6 * T_f, 12 * (T_f ** 2), 20 * (T_f ** 3)]])

        Vx_i = vel_n * math.cos(theta4)
        Vx_f = V_f * math.cos(theta_4)
        Vy_i = vel_n * math.sin(theta4)
        Vy_f = V_f * math.sin(theta_4)
        Ax_i = acc_n * math.cos(theta4) - vel_n * math.sin(theta4) * omega_n
        Ax_f = A_f * math.cos(theta_4) - V_f * math.sin(theta_4) * W_f
        Ay_i = acc_n * math.sin(theta4) + vel_n * math.cos(theta4) * omega_n
        Ay_f = A_f * math.sin(theta_4) + V_f * math.cos(theta_4) * W_f

        xt_ff = xq[index_no + 1]
        yt_ff = yq[index_no + 1]

        g_matrix1 = np.array([x_init, Vx_i, Ax_i, xt_ff, Vx_f, Ax_f])
        g_matrix2 = np.array([y_init, Vy_i, Ay_i, yt_ff, Vy_f, Ay_f])

        P_inverse = np.linalg.inv(P)
        coeff_a = np.dot(P_inverse, g_matrix1)
        coeff_b = np.dot(P_inverse, g_matrix2)

        x_diff = np.zeros(num_t + 1)
        y_diff = np.zeros(num_t + 1)
        time_diff = T_initial

        for ff in range(0, num_t + 1):
            x_diff[ff] = np.dot(
                np.array([1, time_diff, time_diff ** 2, time_diff ** 3, time_diff ** 4, time_diff ** 5]),
                coeff_a)
            y_diff[ff] = np.dot(
                np.array([1, time_diff, time_diff ** 2, time_diff ** 3, time_diff ** 4, time_diff ** 5]),
                coeff_b)
            time_diff = time_diff + (T_f - T_initial) / num_t

        curvature1 = curvature_fn(x_diff, y_diff)

        if np.amax(curvature1) <= 0.15:
            x_set[0] = xq[index_no + 1]
            y_set[0] = yq[index_no + 1]
            iteration = iteration + 1
        else:
            index_no = index_no + 1
            j = j + 1
    print("Overtaking done")
    return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_initial, T_f, V_f, A_f, x_set, y_set, P, trajectory, mtang4
