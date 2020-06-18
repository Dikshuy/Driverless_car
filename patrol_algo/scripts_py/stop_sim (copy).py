import rospy
from std_msgs.msg import String
from patrol_messages.msg import *


class StopSimPubs:
  def __init__(self, in_str):
    print "herre"
    self.stop_msg_pubs = rospy.Publisher('stop_sim_topic', StopSim, queue_size=10)
    self.t_input = in_str.split(' ')
    print('\nOperation Initated')    
    stop_msg = StopSim()
    stop_msg.msg_val = self.t_input[0]    
    if stop_msg.msg_val == "STOP":	
	stop_msg.stop_sim = True
    else:
	stop_msg.stop_sim = False
    print "here", stop_msg.msg_val, stop_msg.stop_sim
    self.stop_msg_pubs.publish(stop_msg)
    print "here1"

if __name__== '__main__':
  print("starting")
  rospy.init_node('initialize_message_test_node', anonymous=True)

  while not rospy.is_shutdown():
    rospy.sleep(0.5)
    try:
      in_str = raw_input("Input message \'STOP\' to stop simulation: ")
      print "in_str"
      operation_msg_ = StopSimPubs(in_str)
    except Exception as e:
      rospy.logerr(e)
  rospy.spin()
