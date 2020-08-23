#!/usr/bin/env python

from __future__ import division
import math
import rospkg
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
from std_msgs.msg import String
from patrol_messages.msg import *
robot_ids = []
yaw, xact , yact, vact, wact =  0.0, 0.0, 0.0, 0.0, 0.0
x_coord = []
y_coord = []
index = 0
height = 0.2
#******************************************************************************** CAIR MAP Segment *************************************************************************
dirname = rospkg.RosPack().get_path('webots_ros')

#Read offset values from net file.
net_file = dirname + '/worlds/CAIR_mod_net/sumo.net.xml'
with open (net_file) as f:
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
edge_info_file = dirname + '/worlds/CAIR_mod_net/CAIR_mod_edge_info.in'
#Read values from a file corresponding to the edge
with open (edge_info_file) as f1:
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


# This function calculates the curvature of set of values( Array of values)
def mengercurve(xway, yway):
    way = np.zeros([len(xway),1])
    s=1
    area, s11, s21, s31 = 0.0, 0.0, 0.0, 0.0
    while(s<np.size(xway)-1):
        area = 0.5*(xway[s-1]*(yway[s] - yway[s+1]) + xway[s]*(yway[s+1] - yway[s-1]) + xway[s+1]*(yway[s-1]-yway[s]))
        s11= math.sqrt(pow((xway[s-1]-xway[s]),2)+ pow((yway[s-1] - yway[s]),2))
        s21= math.sqrt(pow((xway[s+1]-xway[s]),2) + pow((yway[s+1] - yway[s]),2))
        s31= math.sqrt(pow((xway[s-1]-xway[s+1]),2) + pow((yway[s-1] - yway[s+1]),2))
        way[s,0] = (4*area)/(s11*s21*s31)
        s = s+1
    way[0,0] = way[1,0]
    way[np.size(xway)-1,0] = way[np.size(xway)-2,0]
    # print (way)
    return way
#************************************* Velocity Profile *****************************************************************************************************
# Velocity Profile calculates the estimated time at each waypoints and also it calculates the velocities, accelerations, angular velocities at each waypoint
def velocity(xv, yv, v_init, a_init, t_init, state):
	global follow_dis, critical_dis, VMAX, v_obs
	velv = np.zeros([np.size(xv),1])         # 6x1
	tim = np.zeros([np.size(xv),1])          # 6x1
	a_long = np.zeros([np.size(xv),1])       # 6x1
	w = np.zeros([np.size(xv),1])            # 6x1
	a_lat_max = 0.2
	a_long_max= 0.5
	velv[0,0] = v_init   # Initial values
	tim[0,0] = t_init
	a_long[0,0] = a_init
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
	w[0,0] = kkk[0,0]*v_init
	i=1
	while(i<(np.size(xv))):
	    if(kkk[i,0]!=0):                                           # This loop will be for Curve Road because for curve road the curvature value is non-zero
	        v_all = min( v_max, math.sqrt( a_lat_max/abs(kkk[i, 0])) )
	        temp = math.sqrt(max( pow(velv[i-1,0],2) + 2*acc_long*max(math.sqrt(pow((xv[i-1,0]-xv[i,0]),2)+pow((yv[i-1,0] - yv[i,0]),2)), 0), 0))
	        velv[i,0] = max(min(temp, v_all),0)
	        temp1 = (pow(velv[i,0],2) - pow(velv[i-1,0],2))/(2*math.sqrt(pow((xv[i-1,0] - xv[i,0]), 2) + pow(( yv[i-1,0] - yv[i,0]) ,2)))
	        a_long[i,0] = min(temp1, acc_long)

	        w[i,0] = kkk[i,0]*velv[i,0]
	        if(velv[i,0] == velv[i-1,0] and velv[i,0]==0):
	            tim[i,0]= tim[i-1,0] + 1
	        elif(velv[i,0] == velv[i-1] and velv[i]!=0):
	            tim[i,0] = tim[i-1,0] + ((math.sqrt(pow((xv[i-1,0] - xv[i,0]),2)  + pow((yv[i-1,0]-yv[i,0]),2) ))/velv[i-1,0])
	        else:
	            tim[i,0] = (tim[i-1,0] + ((velv[i,0] - velv[i-1,0])/a_long[i,0]))


	    else:
	        temp = math.sqrt(max( pow(velv[i-1,0],2) + 2*acc_long*max(math.sqrt(pow((xv[i-1,0]-xv[i,0]),2)+pow((yv[i-1,0] - yv[i,0]),2)), 0), 0))
	        velv[i,0] = max(min(temp, v_max),0)
	        temp1 = (pow(velv[i,0],2) - pow(velv[i-1,0],2))/(2*math.sqrt(pow((xv[i-1,0] - xv[i,0]), 2) + pow(( yv[i-1,0] - yv[i,0]) ,2)))
	        a_long[i,0] = min(temp1, acc_long)
	        w[0, 0] = 0
	        w[i,0] = 0

	        if(velv[i,0] == velv[i-1,0] and velv[i,0]==0):
	            tim[i,0]= tim[i-1,0] + 1
	        elif(velv[i,0] == velv[i-1] and velv[i]!=0):
	            tim[i,0] = tim[i-1,0] + ((math.sqrt(pow((xv[i-1,0] - xv[i,0]),2)  + pow((yv[i-1,0]-yv[i,0]),2) ))/velv[i-1,0])
	        else:
	            tim[i,0] = (tim[i-1,0] + ((velv[i,0] - velv[i-1,0])/a_long[i,0]))
	    i = i+1
	velv[0,0] = v_init
	tim[0,0] = t_init
	a_long[0,0] = a_init

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
#**********************************************************************************************************************************************
# For single value calculation
# this function calculates the curvature value of each point
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

def callback_model_name(robot_spawn_msg):
	robot_ids.append(robot_spawn_msg.robot_id)
	key_val = robot_spawn_msg.edge_id
	x_coord = edge_dict[key_val]['x_values']
	y_coord = edge_dict[key_val]['y_values']
	for i in range(np.size(x_coord)):
		pos = [round(-x_coord[i]  + xOffset , 2), height, y_coord[i] - yOffset]
		xp.append(pos[2])
		yp.append(pos[0])
	xact = xp[0]
	yact = yp[0]
	ang_radians = math.atan2(xp[1]-xp[0], yp[1]-yp[0])
	yaw = ang_radians

	global x, y
	x = np.zeros([np.size(xp), 1])
	y = np.zeros([np.size(yp), 1])
	for l in range(np.size(x)):
		x[l,0] = xp[l]                  # array of x
		y[l,0] = yp[l]                  # array of y

	#*************************************************************************************************************************************************************
	global follow_dis, critical_dis, VMAX, v_obs, num_foc, epsilon
	follow_dis, critical_dis, VMAX, v_obs = 10, 10, 5.0, 0
	num_foc = 10
	epsilon = 1e-05
	global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur, dyn_x, dyn_y, c1, c2, c3
	dyn_x = x[np.size(x) - 1]
	dyn_y = y[np.size(y) - 1]
	x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
	################## Controller gains###################
	c1, c2, c3 = 2, 0.01, 1.1
	#*********************************************************************************************************************************************


#*************************************************************************************************************************************************************************
if __name__ == "__main__":
	rospy.init_node('check',anonymous=True)
    rospy.Subscriber('spawn_robot', robot_spawn, callback_model_name)
	while len(robot_ids) ==0:
		print "waiting to establish connection"
	print xact, yact
	x_ref_old, y_ref_old = xact, yact
	A_init, t_init, v_init = 0.0, 0.0, 0.0
    rospy.Subscriber('/{}/odom'.format(robot_ids[0]), Odometry, datagps)
	pub_speed = rospy.Publisher('/{}/cmd_vel'.format(robot_ids[0]),Twist,queue_size=10)
	pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100)
	graph_data = Float64MultiArray()
	speed_car = Twist()
	time.sleep(1.0)

	while(index < np.size(x)):
		if index <= np.size(x) -8:
			state = 0
			xr, yr, vel, t, a_long, w= velocity(x[index:index+3], y[index:index+3], v_init, A_init, t_init, state)

		elif index == np.size(x) - 7:
			# state = 3
			# xr, yr, vel, t, a_long, w= velocity(x[index:index+2], y[index:index+2], v_init, A_init, t_init, state)
			break

		m_tang = ( y[index+1, 0] - y[index] )/ ( x[index + 1] - x[index] )

		if(abs(x[index+1] - x[index]) < 0):
			theta = math.pi + math.atan(m_tang)  # This angle is required for calculation of initial values of velocities, acceleration in x, y direction
		else:
			theta = math.atan(m_tang)
		#print "here4......."

		m_tang2 = ( (y[index+2] - y[index+1]) / ( x[index+2] - x[index + 1]) )

		if(abs(x[index +1] - x[index]) < 0):
			th_2 = math.pi + math.atan( m_tang2 )   # This angle is required for calculation of final values of velocities, acceleration in x, y direction
		else:
			th_2 = math.atan( m_tang2 )
		x_set_init, xt_f = xr[0], xr[1]
		y_set_init, yt_f = yr[0], yr[1]
		V_i, V_f, T_i, T_f = vel[0], vel[1], 0, round(float(t[1] - t[0]), 4)
		W_i, W_f, A_i, A_f = w[0], w[1], a_long[0], a_long[1]
		vx_i, vx_f = V_i*math.cos(theta), V_f*math.cos(th_2)
		vy_i, vy_f = V_i*math.sin(theta), V_f*math.sin(th_2)
		Ax_i, Ax_f = ( A_i*math.cos(theta) - V_i*math.sin(theta)*W_i), ( A_f*math.cos(th_2) - V_f*math.sin(th_2)*W_f )
		Ay_i, Ay_f = ( A_i*math.sin(theta) + V_i*math.cos(theta)*W_i ), (  A_f*math.sin(th_2) + V_f*math.cos(th_2)*W_f )
		g1 = np.matrix([ [round(float(x_set_init),4)], [round(float(vx_i),4)], [round(float(Ax_i),4)], [round(float(xt_f),4)], [round(float(vx_f),4)], [round(float(Ax_f),4)] ], dtype='float')
		g2 = np.matrix([ [round(float(y_set_init),4)], [round(float(vy_i),4)], [round(float(Ay_i),4)], [round(float(yt_f),4)], [round(float(vy_f),4)], [round(float(Ay_f),4)] ], dtype='float')
		P = np.matrix([[1,T_i,pow(T_i,2),pow(T_i,3),pow(T_i,4),pow(T_i,5)],
	                  [0,1,2*T_i, 3*pow(T_i,2),4*pow(T_i,3),5*pow(T_i,4)],
	                  [0,0,2,6*T_i,12*pow(T_i,2),20*pow(T_i,3)],
	                  [1,T_f,pow(T_f,2), pow(T_f,3), pow(T_f,4),pow(T_f,5)],
	                  [0,1,2*T_f,3*pow(T_f,2), 4*pow(T_f,3), 5*pow(T_f,4)],
	                  [0,0,2,6*T_f,12*pow(T_f,2),20*pow(T_f,3)]])
		a_coff = np.matmul(np.linalg.inv(P),g1)   # coeffients of equation
		b_coff = np.matmul(np.linalg.inv(P),g2)
		xd, yd, td, TD = np.zeros([num_foc+1,1]), np.zeros([num_foc+1,1]), T_i, np.zeros([num_foc+1,1])
		for f in range(num_foc+1):
			xd[f,0] = np.dot(np.matrix([1,td,pow(td,2), pow(td,3), pow(td,4), pow(td,5)], dtype='float'), a_coff)
			yd[f,0] = np.dot(np.matrix([1,td,pow(td,2), pow(td,3), pow(td,4), pow(td,5)], dtype='float'), b_coff)
			td = td + (T_f - T_i)/num_foc
			TD[f,0] = td
		tdd = 0
		while(tdd < (T_f)):
			tdd = (rospy.get_time() - t[0])
			mu1 = np.matrix([0,1,2*tdd, 3*pow(tdd,2), 4*pow(tdd,3), 5*pow(tdd,4)])
			matu1 = np.dot(mu1, a_coff)
			matu2 = np.dot(mu1, b_coff)
			vref = max(min(math.sqrt( pow(matu1,2) + pow(matu2,2) ),VMAX) , 0)
			xref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3), pow(tdd,4), pow(tdd,5)]), a_coff)
			yref = np.dot(np.matrix([1, tdd, pow(tdd,2), pow(tdd,3), pow(tdd,4), pow(tdd,5)]), b_coff)
			x_ref_new = xref
			y_ref_new = yref
			# print (float(xref), float(yref))
			theta_ref_new = math.atan((y_ref_new - y_ref_old)/(x_ref_new - x_ref_old))   # Reference Theta angle Value will come from wrapper code
			if theta_ref_new<0:
				theta_ref_new += 2*math.pi
			theta_act = yaw        # Actual theta angle Value will come from wrapper code
			if theta_act< 0:
				theta_act += 2*math.pi
			m_out = menger(xref, yref)
			omega_ref = m_out*vref
		        # print "theta:", theta_act
		        # print "x_reference:", x_ref_new[0], "x_actual",xact
		        # print "y_reference:", y_ref_new[0], "y_actual", yact
		        #print "\n"
			xe = float(math.cos(theta_act)*(x_ref_new - xact) + math.sin(theta_act)*(y_ref_new - yact))   # x_act and y_act will come from wrapper code
			ye = float(-(math.sin(theta_act)*(x_ref_new - xact)) + math.cos(theta_act)*(y_ref_new - yact))
			theta_e = float(theta_ref_new - theta_act)
			if theta_e < -math.pi:
				theta_e+=2*math.pi
			elif theta_e > math.pi:
				theta_e-=2*math.pi
			# ********************************** Controller Equations ***********************************************************************************************************************
			control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))   # Controlled Linear Velocity
			controlled_omega = omega_ref + (((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) # Controlled Angular Velocity
			#********************************************************************************************************************************************************************************
			# print(control_velocity)
			speed_car.linear.x = control_velocity
			speed_car.angular.z = - controlled_omega
			pub_speed.publish(speed_car)
			graph_data.data = [xref, yref, xact, yact]  # data for drawing
			#print float(xe), float(ye), float(theta_e), float(vref), float(control_velocity), float(vact), float(omega_ref), float(controlled_omega), float(wact), float(rospy.get_time())
			pub_graph_data.publish(graph_data)
			theta_ref_old = theta_ref_new
			x_ref_old = x_ref_new
			y_ref_old = y_ref_new
			rospy.sleep(0.064)
		v_init = vel[1]         # Initializations
		A_init = a_long[1]
		t_init = t[1]
		index = index+1
		#print(index)
		# print "here3....."        # Initializations


	decelerate = speed_car.linear.x ** 2 / (2 * math.sqrt( pow( (x[np.size(x) - 6]-dyn_x),2 ) + pow ( (y[np.size(y) - 6]-dyn_y),2 )))
	print("starts decelarating")
	print(decelerate)
	print(speed_car.linear.x)
	for i in range((np.size(x) -6), (np.size(x) - 3)):
		distance = round(math.sqrt( pow( (x[i+2]-x[i]),2 ) + pow ( (y[i+2]-y[i]),2 )), 1)
		final_v = math.sqrt(abs(speed_car.linear.x**2 - 2* decelerate * distance)) #((np.size(x) -6) - (np.size(x) - 1))
		print("final_v")
		speed_car.linear.x = final_v
		print(speed_car.linear.x)
		speed_car.angular.z = - speed_car.linear.x * menger(x[i], y[i])
		time.sleep(0.064)
		pub_speed.publish(speed_car)

	speed_car.linear.x = 0.0
	print("stop")
	speed_car.angular.z = 0.0
	pub_speed.publish(speed_car)
# stopping just after the required point
