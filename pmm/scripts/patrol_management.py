#!/usr/bin/python
"""
Patrol Management Class
This module will implement interfaces with GUI and patrolling algorithm, bookkeeping and unit tests.
"""
__version__ = '0.1'
__author__ = 'Nitin Dhiman'

import rospy
from patrol_messages.msg import *

rospy.init_node('Patrol_Management_Module')

robot_id_state_dict={}
blocked_node_ids=[]
patrol_initialize_seq=-1 #This is a flag for operating on first accurate message only!
node_priority_list=[]

#class Patrol_Management:
'''
    Patrol_Management implements the callbacks and state-machines for managing robot states. 
    It also provides interfaces for calling the patrolling algorithm
'''
    #initialize_param_subs  #ros subscriber to recieve the initialize_patrol messages
    #initialize_patrol_topic_name  #Topic name, in global namespace, on which initialize_patrol message will be received
    #algorithm_name  #name of algorithm to invoke. Default is CBLS
    #robot_id_state_dict 
    #blocked_node_ids
    #node_priority_list 
#flag_initialiazation_#done = False

def initialize_patrol_CB(msg):
        '''
        Callback for initialize
        '''
        init_patrol_msg = Initialize_patrol()
        init_patrol_msg.folder = msg.folder
        init_patrol_msg.graph = msg.graph
        init_patrol_msg.num_priority = msg.num_priority
        init_patrol_msg.priority_nodes = msg.priority_nodes
        init_patrol_msg.time_periods = msg.time_periods
        init_patrol_msg.num_robots = msg.num_robots
        init_patrol_msg.algo = msg.algo
        init_patrol_msg.algo_params = msg.algo_params
        init_patrol_msg.min_time = msg.min_time
        print('\n\nINITIALIZE Patrol CB CALLED')
        print(init_patrol_msg.folder, init_patrol_msg.graph, init_patrol_msg.num_priority, init_patrol_msg.priority_nodes, init_patrol_msg.time_periods, init_patrol_msg.num_robots, init_patrol_msg.algo, init_patrol_msg.algo_params, init_patrol_msg.min_time)
        pmm_to_algo_valid_walks_pub.publish(init_patrol_msg)
        initialize_param_subs.unregister()

    
def robot_initialize_patrol_CB(robot_initialize_patrol_msg):
        '''
        Callback for the topic robot_initial_patrol_topic
        '''
        patrol_initialize_seq = robot_initialize_patrol_msg.header.seq
        path_of_rndf_file = robot_initialize_patrol_msg.path_of_rndf_file
        blocked_node_ids = robot_initialize_patrol_msg.blocked_node_ids
        for r_s in robot_initialize_patrol_msg.robot_states:
            set_robot_state(r_s.robot_id, r_s.robot_state) #todo check for multiple entries for same robot
        algorithm_name = robot_initialize_patrol_msg.algo_type
        for node_priority in robot_initialize_patrol_msg.Priority_Node_Definition:
            node_priority_list.append(node_priority) #todo error hadling when node-id do not exists in the rndf file
        print('\n\nROBOT_INITIALIZE CB CALLED')
        print(patrol_initialize_seq, path_of_rndf_file, blocked_node_ids, algorithm_name, node_priority_list)
        robot_initialize_param_subs.unregister()

def team_update_CB(team_update_msg):
        '''
        Callback for topic team_upate
        '''
        for robot_opr in team_update_msg.robot_operations:
            robot_id = robot_opr.robot_id
            opr = robot_opr.operation
            #TODO call the algorithm to set the robot to this state. Handle the error conditions based on the response
            if opr == robot_operation_assignment.ADD_ROBOT_TO_SYSTEM:
                #TODO if robot do not already exists
                set_robot_state(robot_id, robot_ID_state.IDLE)
            elif opr == robot_operation_assignment.ADD_TO_PATROL:
                #TODO if robot do not already exists in patrol/system
                #TODO inform the alogrithm
                self.set_robot_state(robot_id,robot_ID_state.AUTONOMOUS_PATROLLING)
            elif opr == robot_operation_assignment.TELEOPERATE:
                #TODO
                set_robot_state(robot_id, robot_ID_state.TELEOPERATED)
            elif opr == robot_operation_assignment.REMOVE_FROM_PATROL:
                #TODO Checks
                set_robot_state(robot_id, robot_ID_state.IDLE)
            elif opr == robot_operation_assignment.REMOVE_ROBOT_FROM_SYSTEM:
                #TODO checks 
                #shall delete?
                set_robot_state(robot_id, robot_ID_state.UNAVAILABLE)
        
        print('\n\nTEAM_UPDATE CB CALLED')
        print(robot_id, opr)
        print("STATE OF ROBOT 1: ")
        get_robot_state("robot1")
        print("STATE OF ROBOT 2: ")
        get_robot_state("robot2")
        print("STATE OF ROBOT 3: ")
        get_robot_state("robot3")
        team_update_subs.unregister()
            
def manual_patrol_CB(manual_patrol_msg):
        '''
        Callback for topic manual_patrol_topic
        '''
        robot_id = manual_patrol_msg.robot_id
        patrol_route = manual_patrol_msg.patrol_route
        manual_opr = manual_patrol_msg.manual_operation
        if manual_opr == Manual_patrol.ADD_TO_MANUAL_PATROL:
            #TODO update state
            set_robot_state(robot_id, robot_ID_state.MANUAL_PATROLLING)
        elif manual_opr == Manual_patrol.REMOVE_FROM_MANUAL_PATROL:
            # TODO update state
            set_robot_state(robot_id, robot_ID_state.IDLE)
        remove_route_from_autonomous_patrol = manual_patrol_msg.remove_route_from_autonomous_patrol
    
        print('\n\nMANUAL CB CALLED')
        print(robot_id, patrol_route, manual_opr)
        manual_patrol_subs.unregister()

def control_patrol_run_CB(control_patrol_run_msg):
        '''
        Callback for topic control_patrol_run_topic
        '''
        algo_command = control_patrol_run_msg.algo_command
        print('\n\nPAUSE CB CALLED')
        print(algo_command)
        control_patrol_run_subs.unregister()
        

def set_robot_state(robot_id, robot_state):
        '''
        This function will set/reset the robot. 
        Return value 1 is value set, 2 if value reset
        '''
        return_value=2
        try:
            robot_id_state_dict[robot_id] = robot_state
        except KeyError:
            robot_id_state_dict[robot_id] = robot_state
            return_value=1
        #rospy.logdebug("Changed Robot_id "+robot_id+ " to state "+ robot_state)
        return return_value
    
def get_robot_state(robot_id):
        '''
        This function will return the robot_state w.r.t robot_id
        Return value: 1 if robot_id exists in dictionary else 0
        '''
        return_value=1
        try:
            robot_state = robot_id_state_dict[robot_id]
            print(robot_state)
        except KeyError:
            return_value = 0
        return return_value




#def __init__(self):

#MAIN

print("GUI TO PMM")
#self.initialize_patrol_topic_name = rospy.get_param('initial_patrol_topic_name','initial_patrol_topic_name_DEFAULT')
pmm_to_algo_valid_walks_pub = rospy.Publisher('pmm_to_algo_params', Initialize_patrol, queue_size=10)
robot_initialize_param_subs = rospy.Subscriber('robot_initialize_patrol_topic_name', Robot_Initialize, robot_initialize_patrol_CB)
#self.team_update_topic_name = rospy.get_param('team_update_topic_name','team_update_topic')
team_update_subs = rospy.Subscriber('team_update_topic_name', team_update, team_update_CB)
#self.manual_patrol_topic_name = rospy.get_param('manual_patrol_topic_name', 'manual_patrol_topic')
manual_patrol_subs = rospy.Subscriber('manual_patrol_topic_name', Manual_patrol, manual_patrol_CB)
#self.control_patrol_run_topic_name = rospy.get_param('control_patrol_run_topic_name', 'control_patrol_run_topic')
control_patrol_run_subs = rospy.Subscriber('control_patrol_run_topic_name', Control_patrol_run, control_patrol_run_CB)
initialize_param_subs = rospy.Subscriber('initialize_patrol_topic_name', Initialize_patrol, initialize_patrol_CB)
print("PMM TO ALGO")

rospy.spin()

