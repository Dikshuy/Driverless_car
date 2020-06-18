#!/usr/bin/env python

'''

This is publisher ->msg type patrol_messages/TaskDone.
Intended for testing purpose.
'''

import rospy
from patrol_messages.msg import *

class TestTaskDonePubs:
    def __init__(self):
        try:
            self.task_done_pubs = rospy.Publisher('/task_done', TaskDone, self, queue_size=2)
        except Exception:
            print ("Error in creation of task done publisher")

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        print ('connected')        
        task_done_msg = TaskDone()
        task_done_msg.node_id.append('a')
        task_done_msg.robot_id.append(1)
        task_done_msg.stamp = 10000.0
        self.task_done_pubs.publish(task_done_msg)
     
    # def publish_task_done(task_done):
    #     try:
    #         global task_done_pub
    #         task_done_pub = rospy.Publisher('/task_done',TaskDone, queue_size=10)
    #         print("Resolved name is %s",task_done_pub.resolved_name)
    #     except Exception:
    #         print 'Received an exception in publisher creation'
    #     while not rospy.is_shutdown():
    #         count_connections = task_done_pub.get_num_connections()
    #         if count_connections > 0:            
    #             rospy.loginfo(" Number of live subscribers are: %d",count_connections)
    #             task_done_pub.publish(task_done_msg)            
    #             break

if __name__ == "__main__":
    rospy.init_node('test_task_done_pub_node', anonymous = False)
    # task_done_msg = TaskDone()
    # task_done_msg.node_id.append('a')
    # task_done_msg.robot_id.append(1)
    # task_done_msg.stamp = 10000.0
    # publish_task_done(task_done_msg)
    t = TestTaskDonePubs()
    rospy.spin()

    