#!/usr/bin/env python
import rospy

from messages.msg import Initialize_patrol

rospy.init_node('Patrol_Management_Module')

def callback_sub(msg):

	msg_pub = Initialize_patrol()
	msg_pub.folder = msg.folder
        msg_pub.graph = msg.graph
	msg_pub.num_priority = msg.num_priority
	msg_pub.priority_nodes = msg.priority_nodes
	msg_pub.time_periods = msg.time_periods
	msg_pub.num_robots = msg.num_robots
	msg_pub.algo = msg.algo
	msg_pub.algo_params = msg.algo_params
	msg_pub.min_time = msg.min_time
	pub.publish(msg_pub)
        


sub = rospy.Subscriber('initial_params', Initialize_patrol, callback_sub)
pub = rospy.Publisher('pmm_to_algo_params', Initialize_patrol, queue_size=10)



rospy.spin()
