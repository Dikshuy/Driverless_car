import math

xq = []
yq = []


def initialization1(velocity, acceleration_in_x, w, time, p, indexes, angle):
    global xq, yq
    traj = 1

    v_i = velocity[p]
    v_f = velocity[p + 1]
    t_i = 0
    t_f = time[p + 1] - time[p]
    w_i = w[p]
    w_f = w[p + 1]
    a_i = acceleration_in_x[p]
    a_f = acceleration_in_x[p + 1]

    m_tang2 = (yq[indexes + 2] - yq[indexes + 1]) / (xq[indexes + 2] - xq[indexes + 1])
    if xq[indexes + 1] - xq[indexes] < 0:
        theta2 = math.pi + math.atan(m_tang2)
    else:
        theta2 = math.atan(m_tang2)

    P = [[1, t_i, t_i ** 2, t_i ** 3, t_i ** 4, t_i ** 5],
         [0, 1, 2 * t_i, 3 * (t_i ** 2), 4 * (t_i ** 3), 5 * (t_i ** 4)],
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

    x_set = xq[indexes + 1]
    y_set = yq[indexes + 1]

    return vx_i, vx_f, vy_i, vy_f, ax_i, ax_f, ay_i, ay_f, t_i, t_f, v_f, a_f, x_set, y_set, P, traj
