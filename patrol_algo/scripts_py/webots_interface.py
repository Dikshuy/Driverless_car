import rospy
from std_msgs.msg import String
from patrol_messages.msg import *


class TestOperationMsgPubs:
  def __init__(self, in_str):
    self.robot_oper_msg_pubs = rospy.Publisher('add_remove_bots',rob_webots, queue_size=10)
    self.t_input = in_str.split(' ')

    print('\nOperation Initated')    
    rob_webots_msg = rob_webots()
    
    rob_webots_msg.opr = self.t_input[0]
    rob_webots_msg.rob_id = self.t_input[1]
    if len(self.t_input) == 3:
    	rob_webots_msg.edge_id = self.t_input[2]
    else:
	rob_webots_msg.edge_id =""
    self.robot_oper_msg_pubs.publish(rob_webots_msg)
     
if __name__== '__main__':
  print("starting")
  rospy.init_node('initialize_message_test_node', anonymous=True)

  while not rospy.is_shutdown():
    rospy.sleep(0.5)
    try:
      in_str = raw_input("\nAdd/Remove robot? (Eg: 'add r4' or 'rem r1') : ")
      operation_msg_ = TestOperationMsgPubs(in_str)
    except Exception as e:
      rospy.logerr(e)
  rospy.spin()
