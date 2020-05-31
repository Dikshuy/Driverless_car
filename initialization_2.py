import numpy as np
import math

xq = []
yq = []
dyn1_x = []
dyn1_y = []
V_max = 5
m_tang3 = 0
xf = []
yf = []
x_f = []
y_f = []


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
                       x_f + math.sin(theta3) * l_off, x_f + 1.5 * math.sin(theta3) * l_off]])
    y_set = np.array([[y_f + math.cos(theta3) * l_off, y_f + 0.5 * math.cos(theta3) * l_off, y_f,
                       y_f - math.cos(theta3) * l_off, y_f - 1.5 * math.cos(theta3) * l_off]])

    return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory, itr2, m_perp3, itr4, jj
