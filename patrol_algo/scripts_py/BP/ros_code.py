#!/usr/bin/env python

import rospy
from patrol_messages.msg import *

def valid_walks_done_CB(msg):
    if msg.valid_walks_done:
        print("SUMO Received Start confirmation")

if __name__ == '__main__':
    rospy.init_node('sumo_sim')
    rospy.loginfo("Main started")
    valid_walks_done_sumo = rospy.Subscriber('valid_walks_topic', ValidWalksDone, valid_walks_done_CB)
    # s = Sumo_Sim()
    rospy.spin()
    rospy.loginfo("Main ended")

