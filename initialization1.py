import math

xq = []
yq = []


def initialization1(vel, a_long, w, t, i, index, theta):
    global xq, yq
    traj = 1

    v_i = vel[i]
    v_f = vel[i + 1]
    t_i = 0
    t_f = t[i + 1] - t[i]
    w_i = w[i]
    w_f = w[i + 1]
    a_i = a_long[i]
    a_f = a_long[i + 1]

    m_tang2 = (yq[index + 2] - yq[index + 1]) / (xq[index + 2] - xq[index + 1])
    if xq[index + 1] - xq[index] < 0:
        theta2 = math.pi + math.atan(m_tang2)
    else:
        theta2 = math.atan(m_tang2)

    P = [[1, t_i, t_i ** 2, t_i ** 3, t_i ** 4, t_i ** 5],
         [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
         [0, 0, 2, 6 * t_i, 12 * (t_i ** 2), 20 * (t_i ** 3)],
         [1, t_f, t_f ** 2, t_f ** 3, t_f ** 4, t_f ** 5],
         [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
         [0, 0, 2, 6 * t_f, 12 * (t_f ** 2), 20 * (t_f ** 3)]]

    vx_i = v_i * math.cos(theta)
    vx_f = v_f * math.cos(theta2)
    vy_i = v_i * math.sin(theta)
    vy_f = v_f * math.sin(theta2)
    ax_i = a_i * math.cos(theta) - v_i * math.sin(theta) * w_i
    ax_f = a_f * math.cos(theta2) - v_f * math.sin(theta2) * w_f
    ay_i = a_i * math.sin(theta) + v_i * math.cos(theta) * w_i
    ay_f = a_f * math.sin(theta2) + v_f * math.cos(theta2) * w_f

    x_set = xq[index + 1]
    y_set = yq[index + 1]

    return vx_i, vx_f, vy_i, vy_f, ax_i, ax_f, ay_i, ay_f, t_i, t_f, v_f, a_f, x_set, y_set, P, traj
