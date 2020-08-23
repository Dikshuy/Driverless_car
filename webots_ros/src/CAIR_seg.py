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

#******************************************************************************** CAIR MAP Segment *************************************************************************
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

xp, yp, curvp = [], [], []
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
            #print coord
            #Each x added with xOffset
            x_co.append(float(coord[0]) + xOffset)
            #Each y added with yOffset
            y_co.append(float(coord[1]) + yOffset)
        else:
            edge_dict[key_value]['x_values'] = x_co
            edge_dict[key_value]['y_values'] = y_co

x_coord = []
y_coord = []
key_val = '142865547_2'
x_coord = edge_dict[key_val]['x_values']
y_coord = edge_dict[key_val]['y_values']
height = 0.4
roll = 0.0
pitch = 0.0
theta_error = []
om_a = []
om_r = []
rot = []
time_passed = []
x_v = []
y_v = []

for i in range(np.size(x_coord)):
    pos = [-x_coord[i]  + xOffset, height, y_coord[i] - yOffset]
    xp.append(pos[2])
    yp.append(pos[0])

x = np.zeros(np.size(xp))
y = np.zeros(np.size(yp))

for l in range(np.size(x)):
    x[l] = xp[l]                  # array of x
    y[l] = yp[l]                  # array of y

# plt.plot(xp, yp, "red")
# plt.draw()
#************************************************************************************************************************************************************
global follow_dis, critical_dis, VMAX, v_obs
follow_dis, critical_dis, VMAX, v_obs = 10, 10, 5.0, 0
num_foc = 10
epsilon = 1e-10
global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur, dyn_x, dyn_y
dyn_x = x[np.size(x) - 1]
dyn_y = y[np.size(y) - 1]
x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
integral_error = 0

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
#************************************* Velocity Profile *****************************************************************************************************
# Velocity Profile calculates the estimated time at each waypoints and also it calculates the velocities, accelerations, angular velocities at each waypoint
def velocity(xv, yv, v_init, a_init, t_init, state):
	global follow_dis, critical_dis, VMAX, v_obs
	velv = np.zeros(np.size(xv))         # 6x1
	tim = np.zeros(np.size(xv))          # 6x1
	a_long = np.zeros(np.size(xv))       # 6x1
	w = np.zeros(np.size(xv))            # 6x1
	a_lat_max = 0.2
	a_long_max= 0.5
	velv[0] = v_init   # Initial values
	tim[0] = t_init
	a_long[0] = a_init
	kkk = mengercurve(xv, yv)

	if(state == 0):                                                # No obstacle case
	    v_max = VMAX
	    acc_long = a_long_max                                      # depending on state the velocity and a_long is publish

	elif(state == 1):                                              # Follow a car in a lane
	    v_max = min(v_obs,VMAX)
	    acc = 0.5*((pow(v_max,2) - pow(v_init,2))/(math.sqrt(pow((xv[0] - dyn_x),2) + pow((yv[0]-dyn_y),2)) - follow_dis ))
	    acc_long = min(acc, a_long_max)

	elif(state == 2):                                              # overtacking case
	    v_max = VMAX
	    acc_long = a_long_max

	elif(state == 3):                                              # Emergency Break to stop completely due to pedestrian on road
	    dis = math.sqrt( pow( (xv[0]-dyn_x),2 ) + pow ( (yv[0]-dyn_y),2 )) - critical_dis
	    v_max = 0.0
	    acc_long = max( -pow(v_init,2)/(2*abs(dis)) , -VMAX)

	elif(state == 5):                                              # Horizon is exceeded
	    v_max = VMAX
	    acc_long = a_long_max
	w[0] = kkk[0]*v_init
	i=1
	while(i<(np.size(xv))):
	    if(kkk[i]!=0):                                           # This loop will be for Curve Road because for curve road the curvature value is non-zero
	        v_all = min( v_max, math.sqrt( a_lat_max/abs(kkk[i])) )
	        temp = math.sqrt(max( pow(velv[i-1],2) + 2*acc_long*max(math.sqrt(pow((xv[i-1]-xv[i]),2)+pow((yv[i-1] - yv[i]),2)), 0), 0))
	        velv[i] = max(min(temp, v_all),0)
	        temp1 = (pow(velv[i],2) - pow(velv[i-1],2))/(2*math.sqrt(pow((xv[i-1] - xv[i]), 2) + pow(( yv[i-1] - yv[i]) ,2)))
	        a_long[i] = min(temp1, acc_long)

	        w[i] = kkk[i]*velv[i]
	        if(velv[i] == velv[i-1] and velv[i]==0):
	            tim[i]= tim[i-1] + 1
	        elif(velv[i] == velv[i-1] and velv[i]!=0):
	            tim[i] = tim[i-1] + ((math.sqrt(pow((xv[i-1] - xv[i]),2)  + pow((yv[i-1]-yv[i]),2) ))/velv[i-1])
	        else:
	            tim[i] = (tim[i-1] + ((velv[i] - velv[i-1])/a_long[i]))


	    else:
	        temp = math.sqrt(max( pow(velv[i-1],2) + 2*acc_long*max(math.sqrt(pow((xv[i-1]-xv[i]),2)+pow((yv[i-1] - yv[i]),2)), 0), 0))
	        velv[i] = max(min(temp, v_max),0)
	        temp1 = (pow(velv[i],2) - pow(velv[i-1],2))/(2*math.sqrt(pow((xv[i-1] - xv[i]), 2) + pow(( yv[i-1] - yv[i]) ,2)))
	        a_long[i] = min(temp1, acc_long)
	        w[0] = 0
	        w[i] = 0

	        if(velv[i] == velv[i-1] and velv[i]==0):
	            tim[i]= tim[i-1] + 1
	        elif(velv[i] == velv[i-1] and velv[i]!=0):
	            tim[i] = tim[i-1] + ((math.sqrt(pow((xv[i-1] - xv[i]),2)  + pow((yv[i-1]-yv[i]),2) ))/velv[i-1])
	        else:
	            tim[i] = (tim[i-1] + ((velv[i] - velv[i-1])/a_long[i]))
	    i = i+1
	velv[0] = v_init
	tim[0] = t_init
	a_long[0] = a_init

	return xv, yv, velv, tim, a_long, w

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

#*********************************************  Error plotting ************************************************************
def error_plot(error_in_theta, om_act, om_ref, curv, current_time):
	theta_error.append(error_in_theta)
	time_passed.append(current_time)
	om_a.append(om_act)
	om_r.append(om_ref)
	rot.append(curv)
	# plt.plot(time_passed, theta_error, 'r')
	# plt.plot(time_passed, om_r, 'g')
	# plt.plot(time_passed, om_a, 'b')
	plt.plot(time_passed, rot, 'r')
	plt.xlabel("time")
	# plt.ylabel("omega")
   	plt.ylabel("curvature")
	plt.draw()

#******************************************** reference points plotting****************************************************
def plot_points(x_val, y_val):
	x_v.append(x_val)
	y_v.append(y_val)
	# plt.plot(x_v, y_v, 'b')
	# plt.xlabel("x")
	# plt.ylabel("y")
	plt.draw()
#**********************************************************************************************************************************************
# For single value calculation
# this function calculates the curvature value of each point
def menger(x,y):
    global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur
    x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = x_last, x_cur, x, y_last, y_cur, y
    area = 0.5 * (x_last_2 * (y_last - y_cur) + x_last * (y_cur - y_last_2) + x_cur * (y_last_2 - y_last))
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
        return 4 * area /(s1 * s2 * s3)
#*************************************************************************************************************************************************************************
if __name__ == "__main__":
	global yaw, xact , yact, vact, wact
	yaw, xact, yact, vact, wact = 0.0, -51.107000240310946, 3.9327208170484313, 0.0, 0.0
	index = 0
	iterator = 0
	i = 0
	x_ref_old, y_ref_old = -51.107000240310946, 3.9327208170484313
	A_init, t_init, v_init = 0.0, 0.0, 0.0
	rospy.init_node('check',anonymous=True)
	rospy.Subscriber('/agent_0/odom', Odometry, datagps)
	pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
	pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100)
	graph_data = Float64MultiArray()
	speed_car = Twist()
	# time.sleep(1.0)

	while(index < np.size(x)):
		if index <= np.size(x) -8:
			state = 0
			xr, yr, vel, t, a_long, w= velocity(x[index:index+4], y[index:index+4], v_init, A_init, t_init, state)

		elif index == np.size(x) - 7:
			break

		theta = math.atan2(y[index+1]-y[index], x[index+1]-x[index])     # This angle is required for calculation of initial values of velocities, acceleration in x, y direction

		th_2 = math.atan2(y[index+2]-y[index+1], x[index+2]-x[index+1])   # This angle is required for calculation of final values of velocities, acceleration in x, y direction

		x_set_init, xt_f = xr[0], xr[1]
		y_set_init, yt_f = yr[0], yr[1]
		V_i, V_f, T_i, T_f = vel[0], vel[1], 0, round(float(t[1] - t[0]), 4)
		# print "V_i:", V_i
		# print "V_f:", V_f
		W_i, W_f, A_i, A_f = w[0], w[1], a_long[0], a_long[1]
		vx_i, vx_f = V_i*math.cos(theta), V_f*math.cos(th_2)
		vy_i, vy_f = V_i*math.sin(theta), V_f*math.sin(th_2)

		# Ax_i, Ax_f = ( A_i*math.cos(theta) - V_i*math.sin(theta)*W_i), ( A_f*math.cos(th_2) - V_f*math.sin(th_2)*W_f )
		# Ay_i, Ay_f = ( A_i*math.sin(theta) + V_i*math.cos(theta)*W_i ), (  A_f*math.sin(th_2) + V_f*math.cos(th_2)*W_f )
		# g1 = np.array([ round(float(x_set_init),4), round(float(vx_i),4), round(float(Ax_i),4), round(float(xt_f),4), round(float(vx_f),4), round(float(Ax_f),4) ])
		# g2 = np.array([ round(float(y_set_init),4), round(float(vy_i),4), round(float(Ay_i),4), round(float(yt_f),4), round(float(vy_f),4), round(float(Ay_f),4) ])
		g1 = np.array([float(x_set_init), float(vx_i), float(xt_f), float(vx_f)])
		g2 = np.array([float(y_set_init), float(vy_i), float(yt_f), float(vy_f)])
		P = np.array([[1,T_i,pow(T_i,2),pow(T_i,3)],
			          [0,1,2*T_i, 3*pow(T_i, 2)],
			          [1,T_f,pow(T_f,2), pow(T_f,3)],
			          [0,1,2*T_f, 3*pow(T_f, 2)]])

        	P_inv = np.linalg.inv(P)
	    	a_coff = np.dot(P_inv, g1)
	    	b_coff = np.dot(P_inv, g2)
		xd, yd, td, TD = np.zeros(num_foc+1), np.zeros(num_foc+1), T_i, np.zeros(num_foc+1)
		for f in range(0, num_foc+1):
			xd[f] = np.dot(np.array([1,td,pow(td,2), pow(td,3)]), a_coff)
			yd[f] = np.dot(np.array([1,td,pow(td,2), pow(td,3)]), b_coff)
			td = td + (T_f - T_i)/num_foc
			TD[f] = td
		tdd = 0
		integral_error_t = 0
		integral_error_o = 0
		while(tdd < (T_f)):
			tdd = (rospy.get_time() - t[0])
			mu1 = np.matrix([0,1,2*tdd, 3*pow(tdd,2)])
			matu1 = np.dot(mu1, a_coff)
			matu2 = np.dot(mu1, b_coff)
			vref = max(min(math.sqrt( pow(matu1,2) + pow(matu2,2) ),VMAX) , 0)
			xref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3)]), a_coff)
			yref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3)]), b_coff)
			x_ref_new = xref
			y_ref_new = yref
			x_value = np.array(xref)
			y_value = np.array(yref)
			x_val = x_value.squeeze(-1).squeeze(-1)
			y_val = y_value.squeeze(-1).squeeze(-1)

			theta_ref_new = math.atan2((y_ref_new - y_ref_old),(x_ref_new - x_ref_old))
			theta_act = yaw        # Actual theta angle Value will come from wrapper code

			m_out = menger(xref, yref)

			omega_ref = m_out*vref
			xe = float(math.cos(theta_act)*(x_ref_new - xact) + math.sin(theta_act)*(y_ref_new - yact))   # x_act and y_act will come from wrapper code
			ye = float(-(math.sin(theta_act)*(x_ref_new - xact)) + math.cos(theta_act)*(y_ref_new - yact))
			theta_e = float(theta_ref_new - theta_act)
			if theta_e < -math.pi:
				theta_e+=2*math.pi
			elif theta_e > math.pi:
				theta_e-=2*math.pi

			k_i = 0.000524
			k_p = 0.09
			# ********************************** Controller Equations ***********************************************************************************************************************
			control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))   # Controlled Linear Velocity
			controlled_omega = omega_ref + (((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) # Controlled Angular Velocity
			integral_error_t = integral_error_t + theta_e * tdd
			theta_e = k_i * integral_error_t + k_p * theta_e
			integral_error_o = integral_error_o + (controlled_omega - omega_ref) * tdd
			controlled_omega = omega_ref + 0.0075 * integral_error_o + 0.9 *  (controlled_omega - omega_ref)
			#********************************************************************************************************************************************************************************
			speed_car.linear.x = control_velocity
			speed_car.angular.z = - controlled_omega
			pub_speed.publish(speed_car)
			graph_data.data = [xref, yref, xact, yact]  # data for drawing
			pub_graph_data.publish(graph_data)
			theta_ref_old = theta_ref_new
			x_ref_old = x_ref_new
			y_ref_old = y_ref_new
			a = np.array(controlled_omega)
			ang_vel_act = a.squeeze(-1).squeeze(-1)
			b = np.array(omega_ref)
			ang_vel_ref = b.squeeze(-1).squeeze(-1)
			c = np.array(m_out)
			Curve = c.squeeze(-1).squeeze(-1)
			ang_vel = ang_vel_act - ang_vel_ref
  			error_plot(theta_e, a, b, Curve, rospy.get_time())
  			plot_points(x_val, y_val)
			rospy.sleep(0.064)

		v_init = vel[1]         # Initializations
		A_init = a_long[1]
		t_init = t[1]
		index = index+1
		iterator = iterator + 1
		# print "here3....."        # Initializations

	print("starts decelarating")
	decelerate = speed_car.linear.x ** 2 / (2 * math.sqrt( pow( (x[np.size(x) - 7]-dyn_x),2 ) + pow ( (y[np.size(y) - 7]-dyn_y),2 )))
	distance =  math.sqrt( pow( (x[np.size(x) - 7]-dyn_x),2 ) + pow ( (y[np.size(y) - 7]-dyn_y),2 ))
	i = 0
	for i in range(np.size(x) - 7, np.size(x) -2):
		final_v = math.sqrt(speed_car.linear.x**2 - 2* decelerate * ((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2))
		speed_car.linear.x = final_v
		speed_car.angular.z = - speed_car.linear.x * menger(x[i], y[i])
		time_elasped = speed_car.linear.x / (decelerate * 14)
		time.sleep(time_elasped)
		pub_speed.publish(speed_car)

	speed_car.linear.x = 0.0
	speed_car.angular.z = 0.0
	pub_speed.publish(speed_car)
	print("stop")

plt.show()
