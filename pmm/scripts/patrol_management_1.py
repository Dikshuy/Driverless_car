#!/usr/bin/python
"""
Patrol Management Class
This module will implement interfaces with GUI and patrolling algorithm, bookkeeping and unit tests.
"""
__version__ = '0.1'
__author__ = 'Nitin Dhiman'

import os
import rospy
import time
import pdb
from patrol_messages.msg import *
from patrol_messages.srv import *


class Patrol_Management:
    '''
        Patrol_Management implements the callbacks and state-machines for managing robot states.
        It also provides interfaces for calling the patrolling algorithm
    '''
    def __init__(self):
        self.robot_id_state_dict={}
        self.blocked_node_ids=[]
        self.patrol_initialize_seq=-1 #This is a flag for operating on first accurate message only!
        self.node_priority_list=[]
        self.robot_id_list = []

        self.initialize_param_subs = rospy.Subscriber('initialize_patrol_topic_name', Initialize_patrol, self.initialize_patrol_CB)
        self.pmm_to_algo_valid_walks_pub = rospy.Publisher('pmm_to_algo_params', Initialize_patrol, self, queue_size=10)
        self.robot_initialize_param_subs = rospy.Subscriber('robot_initialize_patrol_topic_name', Robot_Initialize, self.robot_initialize_patrol_CB)
        self.team_update_subs = rospy.Subscriber('team_update_topic_name', team_update, self.team_update_CB)
        self.manual_patrol_subs = rospy.Subscriber('manual_patrol_topic_name', Manual_patrol, self.manual_patrol_CB)
        self.control_patrol_run_subs = rospy.Subscriber('control_patrol_run_topic_name', Control_patrol_run, self.control_patrol_run_CB)
        self.battery_level_to_gui_pub = rospy.Publisher('battery_level_to_gui_topic', Robot_Initialize, queue_size=10)
        self.battery_level_subs = rospy.Subscriber('battery_level_topic', Robot_Initialize, self.battery_level_CB)


        # time.sleep(10)
        # self.next_task_obj = NextTaskClass()
        # del self.next_task_obj
        tpbp_to_pmm_service = rospy.Service('tpbp_to_pmm', NextTaskSrv, self.tpbp_pmm_sumo)
        sumo_to_pmm_service = rospy.Service('sumo_to_pmm', TaskDoneSrv, self.sumo_pmm_tpbp)
        #pmm_service.spin()

    def tpbp_pmm_sumo(self, request):
        print('\n\nNEXT TASK RECEIVED FROM TPBP_ONLINE CALLED\n\n')
        print(request.task, request.robot_id)

        try:
            rospy.wait_for_service('pmm_to_sumo')
            pmm_to_sumo_proxy = rospy.ServiceProxy('pmm_to_sumo', NextTaskSrv)
            try:
                print(pmm_to_sumo_proxy(request.task, request.robot_id))
                print("PMM Sent to SUMO")
            except Exception as e:
                rospy.logerr(e)
        except Exception as e:
            rospy.logerr(e)

        return NextTaskSrvResponse("Next Task Params Received by PMM!!!")

    def sumo_pmm_tpbp(self, request):
        print('\n\nTASK DONE RECEIVED FROM SUMO CALLED\n\n')
        print(request.stamp, request.node_id, request.robot_id)

        try:
            rospy.wait_for_service('pmm_to_tpbp')
            pmm_to_tpbp_proxy = rospy.ServiceProxy('pmm_to_tpbp', TaskDoneSrv)
            try:
                print(pmm_to_tpbp_proxy(request.stamp, request.node_id, request.robot_id))
                print("PMM Sent to TPBP")
            except Exception as e:
                rospy.logerr(e)
        except Exception as e:
            rospy.logerr(e)

        return TaskDoneSrvResponse("Task Done Params Received by PMM!!!")


    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        print('\n\nINITIALIZE Patrol PUBLISHER CALLED')
        print(self.init_patrol_msg.folder, self.init_patrol_msg.graph, self.init_patrol_msg.num_priority, self.init_patrol_msg.priority_nodes, self.init_patrol_msg.time_periods, self.init_patrol_msg.num_robots, self.init_patrol_msg.algo, self.init_patrol_msg.algo_params, self.init_patrol_msg.min_time)
        self.pmm_to_algo_valid_walks_pub.publish(self.init_patrol_msg)

    def initialize_patrol_CB(self, msg):
        '''
        Callback for initialize
        '''
        self.init_patrol_msg = Initialize_patrol()
        self.init_patrol_msg.folder = msg.folder
        self.init_patrol_msg.graph = msg.graph
        self.init_patrol_msg.num_priority = msg.num_priority
        self.init_patrol_msg.priority_nodes = msg.priority_nodes
        self.init_patrol_msg.time_periods = msg.time_periods
        self.init_patrol_msg.robot_ids = msg.robot_ids
        self.robot_id_list = self.init_patrol_msg.robot_ids.split(' ')
        self.init_patrol_msg.num_robots = msg.num_robots
        self.init_patrol_msg.algo = msg.algo
        self.init_patrol_msg.algo_params = msg.algo_params
        self.init_patrol_msg.min_time = msg.min_time
        print('\n\nINITIALIZE Patrol CB CALLED')
        print(self.init_patrol_msg.folder, self.init_patrol_msg.graph, self.init_patrol_msg.num_priority, self.init_patrol_msg.priority_nodes, self.init_patrol_msg.time_periods, self.init_patrol_msg.robot_ids, self.init_patrol_msg.num_robots, self.init_patrol_msg.algo, self.init_patrol_msg.algo_params, self.init_patrol_msg.min_time)
        self.initialize_param_subs.unregister()


    def robot_initialize_patrol_CB(self, robot_initialize_patrol_msg):
        '''
        Callback for the topic robot_initial_patrol_topic
        '''
        self.patrol_initialize_seq = robot_initialize_patrol_msg.header.seq
        self.path_of_rndf_file = robot_initialize_patrol_msg.path_of_rndf_file

        #TODO call the code for generation of walks given the rndf file.

        self.blocked_node_ids = robot_initialize_patrol_msg.blocked_node_ids
        for r_s in robot_initialize_patrol_msg.robot_states:
            self.set_robot_state(r_s.robot_id, r_s.robot_state) #todo check for multiple entries for same robot
        algorithm_name = robot_initialize_patrol_msg.algo_type
        for node_priority in robot_initialize_patrol_msg.Priority_Node_Definition:
            self.node_priority_list.append(node_priority) #todo error hadling when node-id do not exists in the rndf file
        print('\n\nROBOT_INITIALIZE CB CALLED')
        print(self.patrol_initialize_seq, self.path_of_rndf_file, self.blocked_node_ids, algorithm_name, self.node_priority_list)
        self.robot_initialize_param_subs.unregister()

    def team_update_CB(self, team_update_msg):
        '''
        Callback for topic team_upate
        '''
        print("\nCallback for topic team_upate")
        robot_id = ""
        for robot_opr in team_update_msg.robot_operations:
            robot_id = robot_opr.robot_id
            opr = robot_opr.operation
            start_node = robot_opr.start_node_id

            #TODO  Handle the error conditions based on the response: call the algorithm to set the robot to this state.
            if opr == robot_operation_assignment.ADD_ROBOT_TO_SYSTEM:
                #TODO if robot do not already exists
                self.set_robot_state(robot_id, robot_ID_state.IDLE)
            elif opr == robot_operation_assignment.ADD_TO_AUTONOMOUS_PATROL:
                rem_after_check = robot_opr.rem_check
                if robot_id not in self.robot_id_list:
                    self.robot_id_list.append(robot_id)
                    self.set_robot_state(robot_id,robot_ID_state.AUTONOMOUS_PATROLLING)
                else:
                    print("Robot id already exists!")
            elif opr == robot_operation_assignment.TELEOPERATE:
                #TODO response to GUI to be done
                self.set_robot_state(robot_id, robot_ID_state.TELEOPERATED)
            elif opr == robot_operation_assignment.REMOVE_FROM_AUTONOMOUS_PATROL:
                rem_after_check = robot_opr.rem_check
                if robot_id in self.robot_id_list:
                    #TODO Checks
                    self.robot_id_list.remove(robot_id)
                    self.set_robot_state(robot_id, robot_ID_state.IDLE)
                else:
                    print("Robot id does not exist to remove!")
            elif opr == robot_operation_assignment.REMOVE_FROM_AUTONOMOUS_PATROL_AFTER_COMPLETION:
                rem_after_check = robot_opr.rem_check
                if robot_id in self.robot_id_list:
                    self.robot_id_list.remove(robot_id)
                    self.set_robot_state(robot_id, robot_ID_state.IDLE)
                else:
                    print("Robot id does not exist to remove!")
            elif opr == robot_operation_assignment.REMOVE_ROBOT_FROM_SYSTEM:
                #TODO checks
                #shall delete?
                self.set_robot_state(robot_id, robot_ID_state.UNAVAILABLE)

        #team_update_pub.publish(self.robot_id_list)
        rospy.wait_for_service('team_update_to_tpbp')
        self.team_update_to_tpbp_proxy = rospy.ServiceProxy('team_update_to_tpbp', TeamUpdateRobotSrv)
        try:
            print(self.team_update_to_tpbp_proxy(self.robot_id_list, robot_id, start_node, rem_after_check))
        except Exception as e:
            print("Exception Occured during team_update: "+ str(e))

        #print('\n\nTEAM_UPDATE CB CALLED')
        #print(robot_id, opr)
        #print("STATE OF ROBOT 1: ")
        #self.get_robot_state("robot1")
        #print("STATE OF ROBOT 2: ")
        #self.get_robot_state("robot2")
        #print("STATE OF ROBOT 3: ")
        #self.get_robot_state("robot3")
        #self.team_update_subs.unregister()

    def manual_patrol_CB(self, manual_patrol_msg):
        '''
        Callback for topic manual_patrol_topic
        '''
        robot_id = manual_patrol_msg.robot_id
        patrol_route = manual_patrol_msg.patrol_route
        manual_opr = manual_patrol_msg.manual_operation
        if manual_opr == Manual_patrol.ADD_TO_MANUAL_PATROL:
            #TODO update state
            self.set_robot_state(robot_id, robot_ID_state.MANUAL_PATROLLING)
        elif manual_opr == Manual_patrol.REMOVE_FROM_MANUAL_PATROL:
            # TODO update state
            self.set_robot_state(robot_id, robot_ID_state.IDLE)
        remove_route_from_autonomous_patrol = manual_patrol_msg.remove_route_from_autonomous_patrol

        print('\n\nMANUAL CB CALLED')
        print(robot_id, patrol_route, manual_opr)
        self.manual_patrol_subs.unregister()

    def control_patrol_run_CB(self, control_patrol_run_msg):
        '''
        Callback for topic control_patrol_run_topic
        '''
        algo_command = control_patrol_run_msg.algo_command
        print('\n\nPAUSE CB CALLED')
        print(algo_command)
        self.control_patrol_run_subs.unregister()

    def battery_level_CB(self, battery_level_msg):
        '''
        Callback for topic battery_level_topic
        '''
        for i in battery_level_msg.robot_states:
            robot_id = i.robot_id
            robot_state = i.robot_state
            if robot_state == robot_ID_state.MOVING_TOWARDS_CHARGING:
                # print("Robot " + robot_id + "'s battery is low. Moving to Charging Zone")
                self.set_robot_state(robot_id, robot_state)
                self.battery_level_to_gui_pub.publish(battery_level_msg)
            elif robot_state == robot_ID_state.IN_CHARGING_ZONE:
                # print("Robot " + robot_id + " is in Charging Zone. Charging...")
                self.set_robot_state(robot_id, robot_state)
                self.battery_level_to_gui_pub.publish(battery_level_msg)
            elif robot_state == robot_ID_state.AUTONOMOUS_PATROLLING:
                # print("Robot " + robot_id + " is Charged")
                self.set_robot_state(robot_id, robot_state)
                self.battery_level_to_gui_pub.publish(battery_level_msg)

    def set_robot_state(self, robot_id, robot_state):
        '''
        This function will set/reset the robot.
        Return value 1 is value set, 2 if value reset
        '''
        return_value=2
        try:
            self.robot_id_state_dict[robot_id] = robot_state
        except KeyError:
            self.robot_id_state_dict[robot_id] = robot_state
            return_value=1
        #rospy.logdebug("Changed Robot_id "+robot_id+ " to state "+ robot_state)
        return return_value

    def get_robot_state(self, robot_id):
        '''
        This function will return the robot_state w.r.t robot_id
        Return value: 1 if robot_id exists in dictionary else 0
        '''
        return_value=1
        try:
            robot_state = self.robot_id_state_dict[robot_id]
            print(robot_state)
        except KeyError:
            return_value = 0
        return return_value


if __name__== '__main__':
    print("starting")
    rospy.init_node('Patrol_Management_Module')
    patrol_management = Patrol_Management()
    rospy.spin()
