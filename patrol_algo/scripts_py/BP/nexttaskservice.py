import rospy
from std_msgs.msg import String
from patrol_messages.srv import *

def next_task(n_list):
   rospy.wait_for_service('tpbp_to_webots')
   try:
     tpbp_to_webots_proxy = rospy.ServiceProxy('tpbp_to_webots', NextTaskSrv) 
     try:
	print (tpbp_to_webots_proxy(n_list), n_list)
     except Exception,e:
        print e
   except rospy.ServiceException, e:
     print rospy.ServiceException, e, "service call failed"
     
if __name__ == "__main__":
   list_trav = ["142865547_3", "-142865547_3", "-142865547_2", "-142865549_2", "-142865549_1", "142865552_2", "-99829039_2"]
   next_task(list_trav)


