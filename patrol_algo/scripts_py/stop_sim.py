import rospy
from std_msgs.msg import String
from patrol_messages.msg import *

if __name__== '__main__':
  print("starting")
  stop_msg_pubs = rospy.Publisher('stop_sim_topic', StopSim, queue_size=10)
  rospy.init_node('initialize_message_test_node', anonymous=True)
	

  while not rospy.is_shutdown():
    rospy.sleep(0.5)
    try:
      in_str = raw_input("Input message \'STOP\' to stop simulation: ")
      stop_msg = StopSim()
      stop_msg.msg_val = in_str.split(' ')[0]
      if stop_msg.msg_val == "STOP":	
	stop_msg.stop_sim = True
      else:
	stop_msg.stop_sim = False
      
      stop_msg_pubs.publish(stop_msg)
      
    except Exception as e:
      rospy.logerr(e)
  rospy.spin()
