#!/usr/bin/env python

import rospy
from patrol_messages.srv import *


def setup_task_done_server():
    rospy.init_node('task_done_server')
    

if __name__ == "__main__":
    setup_task_done_server()