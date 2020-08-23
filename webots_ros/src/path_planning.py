#!/usr/bin/env python

from __future__ import division
import math
import cmath
import rospy
import re
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from numpy import inf
from scipy import *
import time
import matplotlib.pyplot as plt
np.seterr(divide='ignore', invalid='ignore')
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import defaultdict
#********************************************* Declrations ***************************************************************************************************************
#Read offset values from net file.
with open ('/home/dikshant/catkin_ws/src/webots_ros/sumo.net.xml') as f:
    for line in f:
        if "location netOffset=" in line:
            word = line.split(' ')
            for word_l in word:
                if 'netOffset=' in word_l:
                    offset_value = re.findall("\d+\.\d+", word_l)
f.close()
xOffset = float(offset_value[0])
yOffset = float(offset_value[1])

xp, yp = [], []
edge_dict = defaultdict(dict)
#Read values from a file corresponding to the edge
with open ('/home/dikshant/catkin_ws/src/webots_ros/worlds/CAIR_mod_net/CAIR_mod_edge_info.in') as f1:
    for line in f1:
        coord = line.split()
        if len(coord) == 1:
            key_value = coord[0]
            x_co = []
            y_co = []
        elif len(coord) == 2:
            x_co.append(float(coord[0]) + xOffset)
            y_co.append(float(coord[1]) + yOffset)
        else:
            edge_dict[key_value]['x_values'] = x_co
            edge_dict[key_value]['y_values'] = y_co

x_coord = []
y_coord = []
key_val = '99829039_2'
x_coord = edge_dict[key_val]['x_values']
y_coord = edge_dict[key_val]['y_values']
height = 0.4
roll = 0.0
pitch = 0.0

for i in range(np.size(x_coord)):
    pos = [-x_coord[i]  + xOffset, height, y_coord[i] - yOffset]
    xp.append(pos[2])
    yp.append(pos[0])

x = np.zeros(np.size(xp))
y = np.zeros(np.size(yp))
for l in range(np.size(x)):
    x[l] = xp[l]                  # array of x
    y[l] = yp[l]

#*********************************************************************
global xgps, ygps, dyn_x, dyn_y, follow_dis, critical_dis
vinit, ainit, VMAX, num_foc, horizon, v_obs = 0, 0, 5.0, 10, 6, 0
pre_state, state, a_i, t0, x_set_init, y_set_init, lane_status = 0, 0, 1, 0, 0, 0, 0

follow_dis, critical_dis, VMAX, v_obs = 10, 10, 5.0, 0
num_foc = 10
epsilon = 1e-05
global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur, dyn_x, dyn_y
dyn1_x = x[np.size(x)-20]
dyn1_y = y[np.size(y)-20]
x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

# xxxxx, yyyyy , xdu, ydu, TDpp = [], [], [], [], []

a_coefficient_store = np.zeros((6, 5))
b_coefficient_store = np.zeros((6, 5))
xd_n = np.zeros(11)
yd_n = np.zeros(11)
x_store = np.zeros((11, 5))
y_store = np.zeros((11, 5))
time_store = np.zeros((11, 5))
x_store_n = np.zeros((11, 5))
y_store_n = np.zeros((11, 5))
x_set = np.zeros(5)
y_set = np.zeros(5)
T_f = np.zeros(5)
cost_store = np.zeros(5)
T_fin = 0
theta44 = np.zeros(10)
g1 = np.zeros(6)
g2 = np.zeros(6)
a_coefficient_store = np.zeros((6, 5))
b_coefficient_store = np.zeros((6, 5))

################## Controller gains###################
c1, c2, c3 = 2, 0.01, 1.1
#*********************************************************************************************************************************************
# This function calculates the curvature of set of values( Array of values)
def mengercurve(x, y):
    k_matrix = np.zeros(len(x))
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
#************************************* Velocity Profile **************************************************************************************
# Velocity Profile calculates the estimated time at each waypoints and also it calculates the velocities, accelerations, angular velocities at each waypoint
def velocity(x, y, v0, a0, t0, current_state):
    global v_obs, VMAX, critical_dis, follow_dis, dyn_x, dyn_y, dyn1_x, dyn1_y
    length_of_array_x = len(x)
    vel_matrix = np.zeros(length_of_array_x)
    longitudinal_acceleration = np.zeros(length_of_array_x)
    wt = np.zeros(length_of_array_x)
    current_time = np.zeros(length_of_array_x)
    c = mengercurve(x, y)
    a_lat_max = 0.2
    a_long_max = 0.5
    vel_matrix[0] = v0
    current_time[0] = t0
    longitudinal_acceleration[0] = a0

    def zero():  # clear road ahead
        max_v = VMAX
        long_acc = a_long_max
        return max_v, long_acc

    def one():  # follow the vehicle ahead in same lane
        max_v = min(v_obs, VMAX)
        acc = 0.5 * (max_v ** 2 - v0 ** 2) / (math.sqrt((x[0] - dyn_x) ** 2 + (y[0] - dyn_y) ** 2) - follow_dis)
        long_acc = min(acc, a_long_max)
        return max_v, long_acc

    def two():  # overtake the vehicle ahead
        max_v = VMAX
        long_acc = a_long_max
        return max_v, long_acc

    def three():  # emergency braking
        dis = math.sqrt((x[0] - dyn1_x) ** 2 + ((y[0] - dyn1_y) ** 2) - critical_dis)
        max_v = 0.0
        long_acc = max(-v0 ** 2 / (2 * abs(dis)), -VMAX)
        return max_v, long_acc

    def five():  # horizon exceeded
        max_v = VMAX
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

            wt[0] = v0 * c[o]
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

#**********************************************************************************************************************************************
# For single value calculation
def menger(x,y):
    global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur
    x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = x_last, x_cur, x, y_last, y_cur, y
    area = (0.5 * (x_last_2 * (y_last - y_cur) + x_last * (y_cur - y_last_2) + x_cur * (y_last_2 - y_last)))
    s1 = math.sqrt((x_last - x_last_2) ** 2 + (y_last - y_last_2) ** 2)
    s2 = math.sqrt((x_cur - x_last) ** 2 + (y_cur - y_last) ** 2)
    s3 = math.sqrt((x_cur - x_last_2) ** 2 + (y_cur - y_last_2) ** 2)

    # print area, s1, s2, s3
    if s1 <= epsilon:
        return 0.
    if s2 <= epsilon:
        return 0
    if s3 <= epsilon:
        return 0.
    else:
        return 4. * area /(s1 * s2 * s3)
#*********************************************************************************************************************************************
def initialization1(velocity, acceleration_in_x, omega, times, p, indexes, angle):
    global xq, yq, trajectory
    trajectory = 1
    v_i = velocity[p]
    v_f = velocity[p + 1]
    t_i = 0.0
    t_f = times[p + 1] - times[p]
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

    x_set[0] = xq[indexes + 1]
    y_set[0] = yq[indexes + 1]
    print("Following the path")

    return vx_i, vx_f, vy_i, vy_f, ax_i, ax_f, ay_i, ay_f, t_i, t_f, v_f, a_f, x_set, y_set, P_matrix, trajectory

def initialization2(velocity, long_acc, omega, ind, distance, itr2, vd_n, wd_n, ad_n, index, theta, theta4, itr4, jj):
    global m_tang3, xf, yf, x_f, y_f, trajectory
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

    if m_tang3 != 0.0:
        m_perp3 = -1 / m_tang3
    else:
        m_perp3 = - math.inf
    if xq[index + 1] - xq[index] < 0:
        theta3 = math.pi + math.atan(m_tang3)
    else:
        theta3 = math.atan(m_tang3)

    itr2 = itr2 + 1

    if xq[cc] < dyn1_x and yq[cc] == dyn1_y:
        xf = np.array([xq[cc], dyn1_x, xq[cc + 1]])
        yf = np.array([yq[cc], dyn1_y, yq[cc + 1]])
        x_f = xq[cc + 1]
        y_f = yq[cc + 1]
    elif xq[cc] > dyn1_x and yq[cc] == dyn1_y:
        xf = np.array([xq[cc - 1], dyn1_x, xq[cc]])
        yf = np.array([yq[cc - 1], dyn1_y, yq[cc]])
        x_f = xq[cc]
        y_f = yq[cc]
    elif xq[cc] == dyn1_x and yq[cc] == dyn1_y:
        xf = np.array([xq[cc - 1], dyn1_x, xq[cc + 1]])
        yf = np.array([yq[cc - 1], dyn1_y, yq[cc + 1]])
        x_f = xq[cc + 1]
        y_f = yq[cc + 1]
    elif yq[cc] != dyn1_y:
        xf = np.array([xq[cc - 1], xq[cc], xq[cc + 1]])
        yf = np.array([yq[cc - 1], yq[cc], yq[cc + 1]])
        x_f = xq[cc + 1]
        y_f = yq[cc + 1]

    if itr2 > 1:
        V_i = vd_n
    else:
        V_i = velocity[ind]

    V_f = VMAX
    meng = curvature_fn(xf, yf)
    W_i = omega[ind]
    W_f = V_f * meng[1]
    A_i = long_acc[ind]
    A_f = 0
    t_i = 0.0
    t_f = distance / V_i
    P = np.array([[1, t_i, t_i ** 2, t_i ** 3, t_i ** 4, t_i ** 5],
                  [0, 1, 2 * t_i, 3 * (t_i ** 2), 4 * (t_i ** 3), 5 * (t_i ** 4)],
                  [0, 0, 2, 6 * t_i, 12 * (t_i ** 2), 20 * (t_i ** 3)],
                  [1, t_f, t_f ** 2, t_f ** 3, t_f ** 4, t_f ** 5],
                  [0, 1, 2 * t_f, 3 * (t_f ** 2), 4 * (t_f ** 3), 5 * (t_f ** 4)],
                  [0, 0, 2, 6 * t_f, 12 * (t_f ** 2), 20 * (t_f ** 3)]])

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

    x_set = np.array([x_f - math.sin(theta3) * l_off, x_f - 0.5 * math.sin(theta3) * l_off, x_f,
                      x_f + math.sin(theta3) * l_off, x_f + 1.5 * math.sin(theta3) * l_off])
    y_set = np.array([y_f + math.cos(theta3) * l_off, y_f + 0.5 * math.cos(theta3) * l_off, y_f,
                      y_f - math.cos(theta3) * l_off, y_f - 1.5 * math.cos(theta3) * l_off])
    print("Avoiding obstacle")

    return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, t_i, t_f, V_f, A_f, x_set, y_set, P, trajectory, itr2, m_perp3, itr4, jj

def initialization3(x_init, y_init, velocity, vel_n, omega_n, acc_n, index_no):
    global W_f, mtang4, trajectory, Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, V_f, A_f, P, T_f, x_set, y_set
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

        curvature = curvature_fn(x_diff, y_diff)

        if np.amax(curvature) <= 0.15:
            x_set[0] = xq[index_no + 1]
            y_set[0] = yq[index_no + 1]
            iteration = iteration + 1
        else:
            index_no = index_no + 1
            j = j + 1
    print("Overtaking done")
    return Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_initial, T_f, V_f, A_f, x_set, y_set, P, trajectory, mtang4

def cost(xd1, yd1, x_centre, y_centre, phase_chosen):
    global total_cost
    menger_curvature = np.zeros(11)
    menger_curvature[:] = curvature_fn(xd1, yd1)
    if max(menger_curvature[:]) <= 0.15:
        cost1 = 1 / (max(min(np.sqrt(np.add(np.square(xd1 - dyn1_x), np.square(yd1 - dyn1_y)))) - 2, 0.01))
        cost2 = np.sum(np.sqrt(np.add(np.square(x_centre - xd1), np.square(y_centre - yd1))))
        total_cost = 2 * cost1 + 0.5 * cost2

    else:
        total_cost = math.inf

    return total_cost

def overtake(a_store, b_store, store_x, store_y, T_initial, kk, tang_m, perpendicular_m, index_number):
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
            vd_n = max(min(math.sqrt(B * B + C * C), VMAX), 0)
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

def overtake_1(a_store, b_store, store_x, store_y, T_initial, tang_m, perpendicular_m, index_no):
    global T_fin, theta4_n, ad_n, vd_n, wd_n, trajectory
    trajectory = 1
    num_t = 10
    td_n = T_initial

    for tr in range(0, trajectory):
        iterate = 0
        while iterate == 0:
            if tang_m == 0:
                p = np.array([a_store[5, tr],
                              a_store[4, tr],
                              a_store[3, tr],
                              a_store[2, tr],
                              a_store[1, tr],
                              a_store[0, tr] - xq[index_no + 1]])

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
                              b_store[0, tr] - perpendicular_m * a_store[0, tr] + perpendicular_m * xq[index_no + 1] -
                              yq[
                                  index_no + 1]])

                times = np.roots(p)
                for mm in range(0, len(times)):
                    if times[mm].imag == 0:
                        if times[mm] > 0:
                            T_fin = times[mm].real

            for ii in range(0, num_t + 1):
                xd_n[ii] = np.dot([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5], a_store[:, tr])
                yd_n[ii] = np.dot([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5], b_store[:, tr])
                td_n = td_n + (T_fin - T_initial) / num_t

            curv = curvature_fn(xd_n, yd_n)
            A = np.array([0, 1, 2 * T_fin, 3 * T_fin ** 2, 4 * T_fin ** 3, 5 * T_fin ** 4])
            B = np.dot(A, a_store[:, tr])
            C = np.dot(A, b_store[:, tr])
            vd_n = max(min(math.sqrt(B * B + C * C), VMAX), 0)
            wd_n = vd_n * curv[num_t]
            D = np.array([0, 0, 2, 6 * T_fin, 12 * (T_fin ** 2), 20 * (T_fin ** 3)], ndmin=2)
            E = np.dot(D, a_store[:, tr])
            F = np.dot(D, b_store[:, tr])
            ad_n = min(math.sqrt(F * F + E * E), 1.2)

            for mm in range(0, num_t):
                if xq[index_no + 1] - xq[index_no] < 0:
                    theta44[mm] = math.pi + math.atan((yd_n[mm + 1] - yd_n[mm]) / (xd_n[mm + 1] - xd_n[mm]))
                else:
                    theta44[mm] = math.atan((yd_n[mm + 1] - yd_n[mm]) / (xd_n[mm + 1] - xd_n[mm]))

            theta4_n = theta44[num_t - 1]
            store_x[:, tr] = xd_n
            store_y[:, tr] = yd_n
            iterate = iterate + 1
    print("Coming back to main lane")

    return store_x, store_y, vd_n, ad_n, wd_n, theta4_n, T_fin
#**************************************************
def curvature_fn(x,y):
    kk = np.zeros([np.size(x),1])
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    dy = np.gradient(y)
    ddy = np.gradient(dy)
    for i in range(np.size(x)):
        kk[i] = (dx[i]*ddy[i] - dy[i]*ddy[i])/pow((pow(dx[i], 2) + pow(dy[i], 2)),1.5)
    # print (kk)
    return kk

#********************************************** GPS Callback Function ****************************************************************************************************
def datagps(data):
    global yaw, xact , yact, vact, wact
    roll, pitch = 0.0, 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    if data.pose.pose.position.x and data.pose.pose.position.y != 0:
        xact =  data.pose.pose.position.x
        yact = data.pose.pose.position.y
        vact = data.twist.twist.linear.x
        wact = data.twist.twist.angular.z
#*************************************************************************************************************************************************************************
xq = x
yq = y
dyn_x = 0
dyn_y = 0

if __name__ == "__main__":
    global yaw, xact , yact, vact, wact #, xe, ye, theta_e
    yaw, xact, yact, vact, wact = 0.0, 112.66, -1.4, 0.0, 0.0
    index = 0
    iterator = 0
    i = 0
    x_set_init, y_set_init = 112.66, -1.4
    v_init, t_init, A_init = 0.0, 0.0, 0.0
    x_ref_old, y_ref_old = 112.66, -1.4
    # T[0], V[0]  = t_init, v_init
    # id, id2, L, lane_status, horizon, index, state, pre_state, phase = 23, 41, 2.995, 0, 6, 0, 0, 0, 0
    # critical_dis, follow_dis, traj, input1[0], input1[1] = 10, 10, 1, 0, 0
    lane_status, horizon, state, pre_state, phase = 0, 6, 0, 0, 0
    itr2, itr1, vd_n, wd_n, ad_n, th_4, a = 0, 0, 0, 0, 0, 0, 0
    itr4, ll, tsub = 0, 0, 0        # ll is jj here
    # xk = [0, 0, (math.pi/6)]
    critical_dis = 10
    follow_dis = 10
    traj = 1
    rospy.init_node('check',anonymous=True)
    pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100)
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    # pub_graph_data_traj = rospy.Publisher('/graph_data_traj', Float64MultiArray, queue_size= 100)
    speed_car = Twist()
    # graph_data = Float64MultiArray()
    # traj_data = Float64MultiArray() # New publisher for trajectories
    # rospy.sleep(2.0)
    while( index < np.size(x)):
        if state == 5:
            state = pre_state
        xr, yr, vel, t, a_long, w= velocity(x[index:index+horizon], y[index:index+horizon], v_init, ainit, t_init, state)
        i, tempor, pre_state, itr = 0, pre_state, state, 0
        # print(t)
        if state == 2:           # to initialize the time in state 2 this is the temp variable
                ti = t[i]
        while(state == pre_state):
            m_tang = ( y[index+1] - y[index] )/ ( x[index + 1] - x[index] )
            m_perp = -1/m_tang
            xNormalLine = (1/m_perp)*( y - y[index+1])

            theta = math.atan2(y[index+1]-y[index], x[index+1]-x[index])

            if( state != 2 and phase == 0 ):
                Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory  = initialization1( vel, a_long, w, t, i, index, theta)
                # print(P)
            elif( state == 2 and phase == 2):
                # print(x_set_init, y_set_init)
                Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory, value = initialization3(x_set_init, y_set_init, vel, vd_n, wd_n, ad_n, index, th_4) # call for Initialization3 function
                # print (P, "GOLI")
            elif( state == 2 and phase == 1):
                Vx_i, Vx_f, Vy_i, Vy_f, Ax_i, Ax_f, Ay_i, Ay_f, T_i, T_f, V_f, A_f, x_set, y_set, P, trajectory, itr2, m_perp3, itr4, jj = initialization2( vel, a_long, w, i, dist, itr2, vd_n, wd_n, ad_n, index, theta, th_4, itr4, ll)
                xNormalLine1 = (1/m_perp3)*(y - dyn_y) + dyn_x
                # print (x_set)
            for h in range(traj):
                xt_f = x_set[h]
                yt_f = y_set[h]
                if state == 2 and phase == 2:
                    g1 = np.array([x_set_init, Vx_i, Ax_i, xt_f, Vx_f, Ax_f])
                    g2 = np.array([y_set_init, Vy_i, Ay_i, yt_f, Vy_f, Ay_f])
                    P_inv = np.linalg.inv(P)
                    a_coeff = np.dot(P_inv, g1)
                    b_coeff = np.dot(P_inv, g2)

                else:
                    g1 = np.array([x_set_init, Vx_i, Ax_i, xt_f, Vx_f, Ax_f])
                    g2 = np.array([y_set_init, Vy_i, Ay_i, yt_f, Vy_f, Ay_f])
                    P_inv = np.linalg.inv(P)
                    a_coeff = np.dot(P_inv, g1)
                    b_coeff = np.dot(P_inv, g2)

                num_foc_t = 10
                xd = np.zeros(num_foc_t + 1)
                yd = np.zeros(num_foc_t + 1)
                xd_o = np.zeros(num_foc_t + 1)
                yd_o = np.zeros(num_foc_t + 1)
                x_cen = np.zeros(num_foc_t + 1)
                y_cen = np.zeros(num_foc_t + 1)
                TD = np.zeros(num_foc_t + 1)
                vd = np.zeros(num_foc_t + 1)
                wd = np.zeros(num_foc_t + 1)
                td = T_i
                x_point = np.zeros(num_foc_t + 1)
                y_point = np.zeros(num_foc_t + 1)
                x_pnt = np.zeros(num_foc_t + 1)
                y_pnt = np.zeros(num_foc_t + 1)
                for f in range(0, num_foc_t + 1):
                    xd[f] = np.dot(np.array([1, td, td ** 2, td ** 3, td ** 4, td ** 5]), a_coeff)
                    yd[f] = np.dot(np.array([1, td, td ** 2, td ** 3, td ** 4, td ** 5]), b_coeff)

                    td_old = td

                    if state == 2 and phase == 2:
                        td = td + (T_f - T_i) / num_foc_t
                    else:
                        td = td + (T_f - T_i) / num_foc_t

                    TD[f] = td

                    x_cen[f] = np.add(xq[index], np.dot(f, np.divide(np.subtract(xq[index + 1], xq[index]), num_foc_t)))
                    y_cen[f] = np.add(yq[index], np.dot(f, np.divide(np.subtract(yq[index + 1], yq[index]), num_foc_t)))

                if trajectory == 1:
                    x_store[:, 0] = xd
                    y_store[:, 0] = yd
                    x_store_n = x_store
                    y_store_n = y_store
                    a_coefficient_store[:, 0] = a_coeff
                    b_coefficient_store[:, 0] = b_coeff
                    time_store = TD
                elif trajectory == 5:
                    x_store[:, h] = xd
                    y_store[:, h] = yd
                    x_store_n = x_store
                    y_store_n = y_store
                    a_coefficient_store[:, h] = a_coeff
                    b_coefficient_store[:, h] = b_coeff
                    time_store = TD
                d = np.argmin(np.abs(x - dyn_x)) # search for index
                if math.isinf(m_tang):
                    if yq[d] < dyn1_y:
                        var = d + 1
                    elif yq[d] >= dyn1_y:
                        var = d
                else:
                    if xq[d] < dyn1_x:
                        var = d + 1
                    elif xq[d] >= dyn1_x:
                        var = d

                for q in range(0, num_foc_t + 1):
                    # print(math.sqrt((xd[q] - dyn1_x) ** 2 + (yd[q] - dyn1_y) ** 2) )
                    if index < var:  # checking for forward distance & itr1 flag is phase=2 detection(0 means phase~=2 and >0 means phase=2)
                        if math.sqrt((xd[q] - dyn1_x) ** 2 + (yd[q] - dyn1_y) ** 2) < 10.99999999999990:
                            if lane_status == 0:
                                phase = 1
                                state = 2
                                x_pnt[q] = xd[q]
                                y_pnt[q] = yd[q]


                                def non_zero_element(integer):
                                    for no in range(0, len(integer)):
                                        if integer[no] != 0:
                                            return no


                                s = non_zero_element(x_pnt)

                                itr2 = itr2 + 1  # itr2 is the flag just to check immediate detection of obstacle
                                if itr2 == 1:  # when detected
                                    for factor in range(0, s):
                                        x_store[factor] = xd[factor]
                                        y_store[factor] = yd[factor]
                                    for factor2 in range(s, num_foc_t + 1):
                                        x_store[factor2] = xd[s]
                                        y_store[factor2] = yd[s]
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
                if itr2 > 1 and phase == 1:
                    cost_store[h] = total_cost

            if itr2 > 1 and phase == 1:
                number_for_inf_check = 0
                for inf in range(0, len(cost_store)):
                    if cost_store[inf] == math.inf:
                        number_for_inf_check = number_for_inf_check + 1

                if number_for_inf_check == 5:
                    itr4 = 1
                    break
                else:
                    k = np.argmin(cost_store)
                    jj = 0
                [x_store, y_store, vd_n, ad_n, wd_n, theta4, T_add] = overtake(a_coefficient_store,
                                                                                 b_coefficient_store, x_store,
                                                                                 y_store, T_i, k, m_tang, m_perp, index)

            elif phase == 2:
                [x_store, y_store, vd_n, ad_n, wd_n, theta4, T_add] = overtake_1(a_coefficient_store,
                                                                                   b_coefficient_store, x_store,
                                                                                   y_store, T_i, m_tang, m_perp, index)
            if(state == 2 and itr2>1):
                if(phase == 1):
                    x_set_init = x_store[num_foc,k]
                    y_set_init = y_store[num_foc,k]
                    dist = math.sqrt( pow((x_set_init - dyn_x), 2) + pow((y_set_init - dyn_y),2) )
                elif(phase == 2):
                    x_set_init = x_store[num_foc, 0]
                    y_set_init = y_store[num_foc, 0]
                    dist = math.sqrt( pow( (x_set_init - dyn_x),2) + pow((y_set_init - dyn_y),2))
                    itr1 = itr1 + 1
            else:
                x_set_init = x_store[num_foc, 0]
                y_set_init = y_store[num_foc, 0]
                dist = math.sqrt( pow( (x_set_init - dyn_x), 2) + pow((y_set_init - dyn_y),2) )
######################### Print Fianl Output (Trajectories Print) ##########################################################################################################
            # for pp in range(traj):
            #     for qq in range(11):
            #         traj_data.data = [x_store[qq,pp], y_store[qq,pp]]
            #         pub_graph_data_traj.publish(traj_data)
############ Printing Data and sending it to webots also ####################################################################################################################
            # print (float(t[i]), float(t[i+1]))
            if(state == 2 and itr2>1):
                if(phase==1):
                    # print (a_coff_store[:,k])
                    # for p in range(11):
                    #     # print (x_store[p,k])
                    #     graph_data.data = [x_store[p,k], y_store[p,k]]
                    #     pub_graph_data.publish(graph_data)
                    a_coeff_real = a_coefficient_store[:,k]
                    b_coeff_real = b_coefficient_store[:,k]
                    t_i = float(ti)
                    t_f = float(ti)+float(T_add)
                    # print (t_f, t_i, "aaya")
                    #print (t_i, t_f, "state 2")
                elif(phase == 2):
                    if(itr1 == 1):
                        # print (a_coff_store[:,k])
                        # print (t[i],t[i+1], T_add, "phase2-1st")
                        # for rr in range(11):
                        #     # print (x_store[rr,k])
                        #     graph_data.data = [x_store[rr,k], y_store[rr,k]]
                        #     pub_graph_data.publish(graph_data)
                        a_coeff_real = a_coefficient_store[:,k]
                        b_coeff_real = b_coefficient_store[:,k]
                        t_i = float(ti)
                        t_f = float(ti)+float(T_add)
                        # print (t_f, t_i, "aaya idhar")
                       # print (t_i, t_f, "state 2")
                    elif(itr1 >1):
                        # print (a_coff_store[:,0])
                        # print (t[i],t[i+1],T_add, "phase2-2nd")
                        # for rr in range(11):
                        #     # print (x_store[rr,0])
                        #     graph_data.data = [x_store[rr,0], y_store[rr,0]]
                        #     pub_graph_data.publish(graph_data)
                        a_coeff_real = a_coeff_store[:,0]
                        b_coeff_real = b_coeff_store[:,0]
                        t_i = float(ti)
                        t_f = float(ti)+float(T_add)
                        # print (t_f, t_i, "aaya kidhar")
                        #print (t_i, t_f, "state 2")

            else:
                # print (float(rospy.get_time()), float(t[i]), float(t[i+1]))
                if(itr2 == 1):
                    # print (a_coff_store[:,0])
                    # print (t[i], t_store[s,0], t[i+1], "Detected")
                    # for rr in range(11):
                    #         # print (x_store[rr,0])
                    #         graph_data.data = [x_store[rr,0], y_store[rr,0]]
                    #         pub_graph_data.publish(graph_data)
                    a_coeff_real = a_coefficient_store[:,0]
                    # print (a_coff_real)
                    b_coeff_real = b_coefficient_store[:,0]
                    t_i = float(t[i])
                    t_f = float(t[i])+float(time_store[s])
                    # print (t_f, t_i, "gaensh")
                    #print (t_i, t_f, float(t_store[s,0]), "detected Region")
                elif(itr2 == 0):
                    # print (a_coff_store[:,0])
                    # print (t[i], t[i+1], "St. Line")
                    # for rr in range(11):
                    #         # print (x_store[rr,0])
                    #         graph_data.data = [x_store[rr,0], y_store[rr,0]]
                    #         pub_graph_data.publish(graph_data)
                    a_coeff_real = a_coefficient_store[:,0]
                    # print(a_coff_store[:,0])
                    b_coeff_real = b_coefficient_store[:,0]
                    t_i = float(t[i])
                    t_f = float(t[i+1])
                    # print (t_f, t_i, "ganpati bappa")
                    #print (t_i, t_f, "state 0")
            # print (a_coff_real)
            tdd = 0
            # print (t_i, tdd, t_f, "morya")
            while(tdd < (t_f-t_i)):
                tdd = (rospy.get_time() - t_i)
                # print(float(tdd), "time", a_coff_real )
                mu1 = np.matrix([0,1,2*tdd, 3*pow(tdd,2), 4*pow(tdd,3), 5*pow(tdd,4)])
                matu1 = np.dot(mu1, a_coeff_real)
                matu2 = np.dot(mu1, b_coeff_real)
                vref = max(min(math.sqrt( pow(matu1,2) + pow(matu2,2) ),VMAX) , 0)
                xref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3), pow(tdd,4), pow(tdd,5)]), a_coeff_real)
                # xdu.append(xref)
                yref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3), pow(tdd,4), pow(tdd,5)]), b_coeff_real)
                # ydu.append(yref)
                # TDpp.append(tdd)
                x_ref_new = xref
                y_ref_new = yref
                #print (float(xref), float(yref))
                theta_ref_new = math.atan2((y_ref_new - y_ref_old),(x_ref_new - x_ref_old))

                theta_act = yaw        # YAW will come from wrapper

                m_out = menger(xref, yref)
                omega_ref = m_out*vref # It is right or wrong that we need to verify

                xe = float(math.cos(theta_act)*(x_ref_new - xact) + math.sin(theta_act)*(y_ref_new - yact))   # x_act and y_act will come from wrapper code
                ye = float(-(math.sin(theta_act)*(x_ref_new - xact)) + math.cos(theta_act)*(y_ref_new - yact))
                theta_e = float(theta_ref_new - theta_act)

                if theta_e < -math.pi:
                    theta_e+=2*math.pi
                elif theta_e > math.pi:
                    theta_e-=2*math.pi

                control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))
                controlled_omega = omega_ref + (((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2)
                speed_car.linear.x = control_velocity
                speed_car.angular.z = controlled_omega
                pub_speed.publish(speed_car)

                # graph_data.data = [xref, yref, xact, yact]  # data for drawing
                # print float(xe), float(ye), float(theta_e), float(vref), float(control_velocity), float(vact), float(omega_ref), float(controlled_omega), float(wact), float(rospy.get_time())
                # pub_graph_data.publish(graph_data)

                theta_ref_old = theta_ref_new
                x_ref_old = x_ref_new
                y_ref_old = y_ref_new

                rospy.sleep(0.064)
            # tsub = t_i
#******************************************************************************************************************************************************************

# ********************************** State Updation ***************************************************************************************************************

            if(state == 2 and itr2>1):
                if(phase == 1):
                    if(m_tang == 0):
                        if(x_store[num_foc,k] > dyn_x):
                            # print("here it is", x_store[num_foc,k])
                            phase = 2
                            itr1 = itr1 + 1
                    elif(m_tang == inf):
                        if(y_store[num_foc,k]>dyn_y):
                            phase = 2
                            itr1 = itr1 + 1
                    elif(m_tang != 0):
                        if( (x[index + 1]- x[index])< 0 or ( y[index + 1] - y[index] )< 0 ):
                            if(x_store[num_foc, k] < dyn_x and y_store[num_foc, k] < dyn_y):
                                phase = 2
                                itr1 = itr1 + 1
                        else:
                            if(x_store[num_foc,k] > dyn_x):
                                phase = 2
                                itr1 = itr1 + 1
                elif(phase == 2):
                    if(abs(x_store[num_foc, 0] - x[index+1] )< 0.01 and abs(y_store[num_foc,0] - y[index+1])< 0.01 ):  # here we should compare both values but we are not doing so # Problem is here
                        # print (x_store[num_foc, 0], x[index+1], y_store[num_foc,0] , y[index+1])
                        phase = 0
                        state = 0
                        itr = itr + 1

            if(itr2 == 1):              # the time updation when the detection is taking place
                t_init = t_f
            else:
                t_init = t[i+1]
            if(state == 2):
                ti = t_f

            # print (i)
            i = i+1
            if(state != pre_state):
                if(phase == 0 and itr != 0):
                    v_init = vd_n
                    A_init = ad_n
                    index = index + 1
                    itr1 = 0
                itr2 = 0
                break
            if (state == 2):             # if horizon exceeds then t_init should be equal to the t_f during the overtaking
                t_init = t_f
                if(i>=horizon-1):
                    state = 5
            else:
                if(i>=horizon-1):        #  this is in normal state 0
                    state = 5
            index = index + 1
            iterator = iterator + 1
            # if(state == 2 and itr2>1):
            #     if(phase == 1):
            #         if(m_tang == 0):
            #             if(x_store[num_foc,k] > dyn_x):
            #                 # print("here it is", x_store[num_foc,k])
            #                 phase = 2
            #                 itr1 = itr1 + 1
            #         elif(m_tang == inf):
            #             if(y_store[num_foc,k]>dyn_y):
            #                 phase = 2
            #                 itr1 = itr1 + 1
            #         elif(m_tang != 0):
            #             if( (x[index + 1]- x[index])< 0 or ( y[index + 1] - y[index] )< 0 ):
            #                 if(x_store[num_foc, k] < dyn_x and y_store[num_foc, k] < dyn_y):
            #                     phase = 2
            #                     itr1 = itr1 + 1
            #             else:
            #                 if(x_store[num_foc,k] > dyn_x):
            #                     phase = 2
            #                     itr1 = itr1 + 1
            #     elif(phase == 2):
            #         if(abs(x_store[num_foc, 0] - x[index+1] )< 0.01 and abs(y_store[num_foc,0] - y[index+1])< 0.01 ):  # here we should compare both values but we are not doing so # Problem is here
            #             # print (x_store[num_foc, 0], x[index+1], y_store[num_foc,0] , y[index+1])
            #             phase = 0
            #             state = 0
            #             itr = itr + 1
            # i = i+1
            if(phase == 2):
                v_init = vd_n
                A_init = ad_n
            else:
                v_init = V_f
                A_init = A_f
	    #plt.plot(xe, rospy.get_time())
	    #plt.xlabel('error in x')
            #plt.ylabel('time(in sec)')
	    #plt.axis([0, 30, -6, 6])
	    #plt.pause(0.000000000000000000000000001)
            # rospy.sleep(1)
