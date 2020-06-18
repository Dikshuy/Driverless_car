from patrol_messages.srv import *
import rospy

def next_task_service_CB(requ):
    rospy.loginfo('Next task service received')
    print (requ)
    print (requ.task)
    rtn =  NextTaskSrvResponse()
    rtn.res = 'Task Serviced'
    return rtn

def next_task_server():
    rospy.init_node('next_task_server')
    server = rospy.Service('next_task',NextTaskSrv, next_task_service_CB)
    rospy.loginfo('Next task server ready')
    rospy.spin()

if __name__ == "__main__":
    next_task_server()
        
