#!/usr/bin/env python

'''

This is publisher ->msg type patrol_messages/TaskDone.
Intended for testing purpose.
'''

import rospy
from patrol_messages.msg import *


#peer_subscribe(self, topic_name, topic_publish, peer_publish)
#source code 
#callback when a peer has subscribed from a 

class TestNextTaskPublication:
    def __init__(self):
        try:
            self.next_task_pubs = rospy.Publisher('/robot_0/next_task', NextTask,  queue_size=2)
        except Exception:
            print ("Error in creation of next task publisher")

    # def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    #     print ('connected')
    #     next_task_msg = NextTask()
    #     next_task_msg.task.append('0')
    #     next_task_msg.task.append('1')
    #     next_task_msg.task.append('0')
    #     self.next_task_pubs.publish(next_task_msg)

    def publish_next_task(self, next_task):        
        while not rospy.is_shutdown():   
            count_connections = self.next_task_pubs.get_num_connections()
            if count_connections > 0:
                rospy.loginfo (" number of live connections are %d", count_connections)
                self.next_task_pubs.publish(next_task_msg)
                break
            rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node('tpbp_next_task', anonymous = True)
    t = TestNextTaskPublication()
    next_task_msg = NextTask()
    next_task_msg.task.append('0')
    next_task_msg.task.append('1')
    next_task_msg.task.append('0')
    t.publish_next_task(next_task_msg)
    rospy.spin()



