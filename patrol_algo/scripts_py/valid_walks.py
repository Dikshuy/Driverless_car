#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from patrol_messages.msg import *

if __name__== '__main__':
	print("starting")
	pub = rospy.Publisher('valid_walks_topic', ValidWalksDone, queue_size=10)
	rospy.init_node('initialize_message_test_node', anonymous=True)
	msg1 = ValidWalksDone()
	msg1.valid_walks_done = True
	pub.publish(msg1)
	print("END")
	      
	   
