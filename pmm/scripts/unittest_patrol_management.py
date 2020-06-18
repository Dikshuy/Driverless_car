#!/usr/bin/env python
PKG = 'patrol_management_module'
#import patrol_management_module
#from patrol_management_module.scripts import patrol_management
#from patrol_management import Patrol_Management
import rospy
#import setuptools
#print(setuptools.find_packages())
#setuptools.setup(name='patrol_management_module', version='1.0', packages=setuptools.find_packages())

from pmm.scripts.patrol_management import Patrol_Management
from patrol_messages.msg import *
import unittest


class InputInitializeTeam(unittest.TestCase):
    def __init__(self):
        self.pm=None
        self.robot_init_msg_pubs=None 
    

    def setUp(self):
        rospy.init_node('initialize_message_test_node', anonymous=True)
        self.robot_init_msg = Robot_initialize()
        robot_init_msg.header.seq=1
        robot_init_msg.header.stamp = rospy.Time.now()
        robot_init_msg.header.frame_id = 'map'
        robot_init_msg.path_to_rndf_file = '/home/nitin/Documents/rndf.txt'
        robot_init_msg.blocked_node_ids.append('blocked_node1')
        robot_init_msg.blocked_node_ids.append('blocked_node2')
        robot_init_msg.blocked_node_ids.append('blocked_node3')
        ris1 = robot_ID_state()
        ris1.robot_id = 'robot1'
        ris1.robot_state = robot_ID_state.IDLE
        ris2 = robot_ID_state()
        ris2.robot_id = 'robot3'
        ris2.robot_state =robot_ID_state.IN_CHARGING_ZONE
        ris3 =robot_ID_state()
        ris3.robot_state = 'robot2'
        ris3.robot_state = robot_ID_state.IDLE
        robot_init_msg.robot_states.append(ris1)
        robot_init_msg.robot_states.append(ris2)
        robot_init_msg.robot_states.append(ris3)
        pnd1 = priority_node_definition()
        pnd1.node_id='n1'
        pnd1.revisit_time_a=10.0
        pnd1.revisit_time_end=12.0
        pnd2 = priority_node_definition()
        pnd2.node_id='n2'
        pnd2.revisit_time_end=6
        pnd2.revisit_time_a=8
        robot_init_msg.algorithm_type=Robot_initialize.ALGORITHM_TPBP

        self.pm = Patrol_Management()
        #setup a publisher
        self.robot_init_msg_pubs = rospy.Publisher('initial_patrol_topic_name',Robot_initialize, queue_size=10)

	self.team_update_msg = team_update()
	self.robot_oper_msg = robot_operation_assignment()
	robot_oper_msg.robot_id = 'robot1'
	robot_oper_msg.operation = robot_oper_msg.ADD_ROBOT_SYSTEM
	team_update.robot_operations.append(robot_oper_msg)
	self.robot_oper_msg_pubs = rospy.Publisher('team_update_topic_name',team_update, queue_size=10)
	
	self.manual_patrol_route_msg = Manual_patrol()
	manual_patrol_route_msg.robot_id = 'robot2'
	manual_patrol_route_msg.patrol_route = ['n1','n4','n8','n12']
	manual_patrol_route_msg.remove_route_from_autonomous_patrol = True
	manual_patrol_route_msg.manual_operation = manual_patrol_route_msg.ADD_TO_MANUAL_PATROL
	self.manual_patrol_route_msg_pubs = rospy.Publisher('manual_patrol_topic_name',Manual_patrol, queue_size=10)

    def tearDown(self):
        self.pm.dispose()
        self.pm = None
        
    def test_robot_states(self):
        #check for incoming correctness
        self.robot_init_msg_pubs.publish(self.robot_init_msg)
        rospy.sleep(1)
        ri3_robot_state = robot_ID_state.UNAVAILABLE
        ri1_robot_state = robot_ID_state.UNAVAILABLE
        return_value = self.pm.get_robot_state('robot1',ri1_robot_state)
        self.assertEqual(return_value,1)
        return_value = self.pm.get_robot_state('robot2',ri3_robot_state)
        self.assertEqual(return_value,1)
        self.assertEqual(ri1_robot_state, robot_ID_state.IDLE)
        self.assertNotEqual(ri3_robot_state,robot_ID_state.IN_CHARGING_ZONE)
        self.pm.get_robot_state('robot3', ri3_robot_state)
        self.assertEqual(ri3_robot_state,robot_ID_state.IN_CHARGING_ZONE)
        return_value = self.pm.get_robot_state('robot_unknown', ri3_robot_state)
        self.assertEqual(return_value, 10, 'Wrong return value for state access of robot')
       
    def publishers(self):
    	self.robot_oper_msg_pubs.publish(self.team_update_msg)
        rospy.sleep(1)
        self.manual_patrol_route_msg_pubs.publish(self.manual_patrol_route_msg)

    @unittest.skip('TODO')
    def test_blocked_nodes(self):
        a = True

    @unittest.skip('TODO')
    def test_priority_node_list(self):
        b  = False

'''class IncomingDataTestSuite(unittest.TestSuite):
    def __init__(self):
        super(IncomingDataTestSuite,self).__init__()
        self.addTest(InputInitializeTeam())'''

'''
if __name__ == '__main__':
    import rostest
    rostest.rosrun('patrol_management_module', 'pmm_test', 'IncomingDataTestSuite' )'''
