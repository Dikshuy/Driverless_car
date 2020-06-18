#!/usr/bin/env python

import rospy
from patrol_messages.srv import *

class FollowPath:
    def __init__(self):
        #self.task_done_server  = rospy.Service('task_done',TaskDoneSrv, self.task_done_server_CB)
        pass

    def next_task_client(self):
        rospy.wait_for_service('pmm_to_sumo')
        try:
            next_task_proxy = rospy.ServiceProxy('pmm_to_sumo',NextTaskSrv)
            nxt_task = NextTaskSrv()
            robot_id = 'robot_1'
            task = ['1', '0', '1']
            respon = next_task_proxy(task,robot_id)
            return respon.res
        except rospy.ServiceException, e:        
            rospy.logerr('Service call failed')
    
    

if __name__ == '__main__':
    ta = FollowPath() #tpbp will initialize it
    resultant_statement = ta.next_task_client()
    # for i in [1 ,2, 3, 4]:
    #     resultant_statement = ta.next_task_client()
    #     rospy.sleep(1)
    #     rospy.loginfo(resultant_statement)
    #     print(resultant_statement)