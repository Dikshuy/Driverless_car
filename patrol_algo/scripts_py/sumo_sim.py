#!/usr/bin/env python

import os
import sys
import traci
import pdb
import rospy, rospkg
from enum import Enum 
from multiprocessing import Pool
from random import random
import networkx as nx
from patrol_messages.msg import *
from patrol_messages.srv import *

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

flag_already_initialized = False

color_list= ["red", "green", "blue", "yellow", "orange"]
color_dict = { 0:[1,0,0], 1:[0,1,0], 2:[0,0,1,1], 3:[1,0,1,1], 4:[0,1,1,1]}

robot_string_2_robot_num = {}
robot_num_2_robot_string = {}
max_robot_numeral_id = 0
HIGH_NUM = '5000'


class Sumo_Sim:
    def __init__(self):
        self.sub = rospy.Subscriber('pmm_to_algo_params', Initialize_patrol, self.callback_init)
        rospy.sleep(2)          #to receive initial parameters
        self.seq_of_routes = {}
        self.updated = {}
        self.update = False
        self.last_nodes = {}
        self.stopped_already = {}
        self.battery_levels = {'r0':100.0, 'r1':80.0, 'r2':90.0, 'r3':50.0}
        self.team_update_to_sumo_service = rospy.Service('team_update_to_sumo', TeamUpdateRobotSrv, self.team_update_CB)
        self.sumo_service = rospy.Service('pmm_to_sumo', NextTaskSrv, self.pmm_sumo)

        for ith_robot in self.robot_ids:
            self.seq_of_routes[ith_robot] = []
            self.updated[ith_robot] = False
            self.last_nodes[ith_robot] = HIGH_NUM
            self.stopped_already[ith_robot] = True     

        #self.pub = rospy.Publisher('/task_done', TaskDone, queue_size = 10)
        rospy.wait_for_service('sumo_to_pmm')
        self.sumo_to_pmm_proxy = rospy.ServiceProxy('sumo_to_pmm', TaskDoneSrv)

        rospy.loginfo("TraciSim constructon completed")

    def pmm_sumo(self, request):
        print('\n\nNEXT TASK RECEIVED FROM PMM CALLED\n\n')
        print(request.task, request.robot_id)

        route = []
        for i in range(len(request.task) - 1):
            route.append(str(request.task[i]) + "to" + str(request.task[i + 1]))
        if len(self.seq_of_routes[request.robot_id]) != 0:
            route.insert(0, self.seq_of_routes[request.robot_id][-1])
        self.seq_of_routes[request.robot_id] = route
        #rospy.loginfo("Sequence of route. robot_id %s %s", request.robot_id, self.seq_of_routes)
        self.updated[request.robot_id] = True
        self.update = True
        rospy.loginfo("Created data for robot %s", request.robot_id)
        return NextTaskSrvResponse("Next Task Params Received by SUMO!!!")


    def team_update_CB(self, request):
        rospy.loginfo("entered team_update_cb")
        if(len(self.robot_ids) < len(request.robot_id_list)):  
            self.robot_ids = request.robot_id_list                #if a robot is added
            rospy.loginfo("Robot ids after adding: %s", str(self.robot_ids))
            self.seq_of_routes[request.robot_id] = []
            self.updated[request.robot_id] = False
            self.last_nodes[request.robot_id] = HIGH_NUM
            self.stopped_already[request.robot_id] = True  
            self.num_vehicles = self.num_vehicles + 1

        elif(len(self.robot_ids) > len(request.robot_id_list)):     #if a robot is removed
            self.robot_ids = request.robot_id_list              
            rospy.loginfo("Robot ids after removal: %s", str(self.robot_ids)) 
  
        return TeamUpdateRobotSrvResponse("Team Update to Sumo received")

    # def set_up_robot_id_dictionaries(self, robot_ids):
    #     robot_numeral_id=0
    #     for robot_id in robot_ids:
    #         robot_string_2_robot_num[robot_id] = robot_numeral_id
    #         robot_num_2_robot_string[robot_numeral_id] = robot_id
    #         robot_numeral_id = robot_numeral_id + 1
    #     global max_robot_numeral_id
    #     max_robot_numeral_id = robot_numeral_id

    def battery_check_func(self):
        rospy.sleep(40)
        print("Sim_time b4:  " + str(traci.simulation.getTime()))
        while traci.simulation.getTime() <= self.min_time:
            print("Sim_time after:  " + str(traci.simulation.getTime()))
            for itr_robot_i in self.robot_ids:
                self.battery_levels[itr_robot_i] = self.battery_levels[itr_robot_i] - 0.5
                if(self.battery_levels[itr_robot_i]<20.0):     #if battery level of robot is low
                    battery_level_pub = rospy.Publisher('battery_level_topic', Robot_Initialize, queue_size=10)
                    robot_battery_level_msg = Robot_Initialize()
                    r_b_l_msg = robot_ID_state()
                    r_b_l_msg.robot_id = itr_robot_i
                    r_b_l_msg.robot_state = robot_ID_state.MOVING_TOWARDS_CHARGING
                    robot_battery_level_msg.robot_states.append(r_b_l_msg)
                    battery_level_pub.publish(robot_battery_level_msg)
                    rospy.sleep(20)
                    r_b_l_msg.robot_state = robot_ID_state.IN_CHARGING_ZONE
                    robot_battery_level_msg.robot_states.append(r_b_l_msg)
                    battery_level_pub.publish(robot_battery_level_msg)
                    rospy.sleep(20)
                    self.battery_levels[itr_robot_i] = 100.0
                    if(self.battery_levels[itr_robot_i] == 100.0):
                        r_b_l_msg.robot_state = robot_ID_state.AUTONOMOUS_PATROLLING
                        robot_battery_level_msg.robot_states.append(r_b_l_msg)
                        battery_level_pub.publish(robot_battery_level_msg)


    def simulation_startup(self):
        rospy.loginfo("Entered simulation_startup. num_vehicles: %d" ,self.num_vehicles)
        rospy.set_param('done', False)
        #self.set_up_robot_id_dictionaries(self.robot_ids)
        # sumo_startup = ['sumo', '-c', dirname +'/graph_sumo/{}.sumocfg'.format(graph_name)]
        sumo_startup = ['sumo-gui', '-c', self.dirname +'/graph_sumo/{}.sumocfg'.format(self.graph_name)]
        rospy.loginfo(sumo_startup)
        # print 'Click Play button only on ensuring that the patrolling algorithm is running'
        traci.start(sumo_startup)
        rospy.loginfo("Simulation Startup Occured")
        #Click Play Button in the GUI, if GUI
        # pool = Pool(1)                  #CHECKING BATTERY LEVELS
        rospy.sleep(40)
        # p1 = pool.apply_async(self.battery_check_func())       
        # print("Thread started")

        while traci.simulation.getTime() <= self.min_time:
            #print traci.simulation.getTime()
            vehicles = traci.vehicle.getIDList() 
            rospy.loginfo("VEHICLES : %s", str(vehicles))
            for itr_robot_i in self.robot_ids:
                if self.updated[itr_robot_i]:
                    #print 'here', t.updated
                    #robot_numeral_id = robot_string_2_robot_num[itr_robot_i]
                    if itr_robot_i not in vehicles:
                        rospy.loginfo("Seq of routes of %s : %s", itr_robot_i, self.seq_of_routes[itr_robot_i])
                        route_id = itr_robot_i + str(random())
                        traci.route.add(routeID = route_id, edges = self.seq_of_routes[itr_robot_i])
                        traci.vehicle.add(vehID = itr_robot_i, routeID = route_id, typeID = "type1")
                        #traci.vehicle.setColor(str(i),color_dict[i])
                        self.stopped_already[itr_robot_i] = False
                        rospy.loginfo("Added vehicle %s",str(itr_robot_i))
                    else:
                        e = traci.vehicle.getRoute(itr_robot_i)
                        if traci.vehicle.isStopped(itr_robot_i) and e[-1] == self.seq_of_routes[itr_robot_i][0]:
                            traci.vehicle.resume(itr_robot_i)
                            traci.vehicle.setRoute(vehID = itr_robot_i, edgeList = self.seq_of_routes[itr_robot_i])
                            self.stopped_already[itr_robot_i] = False
                        else: 
                            print("\n")   
                print("\n******************************** Seq:  " + itr_robot_i + "--" + str(self.seq_of_routes[itr_robot_i]))
                self.updated[itr_robot_i] = False

            srvs = TaskDoneSrv()
            srvs.stamp = traci.simulation.getTime()
            srvs.node_id = []
            srvs.robot_id = []
            for ith_robot in self.robot_ids:
                vehicles = traci.vehicle.getIDList()
                if ith_robot in vehicles:
                    last_edge = traci.vehicle.getRoadID(ith_robot)
                    print ("\nLast edge: " + str(last_edge))
                    if last_edge.find(':') == -1:
                        last_node = int(last_edge[:last_edge.find('to')])
                        print ("Last node: " + str(last_node))
                        # if traci.vehicle.isStopped(str(i)) and not t.stopped_already[i]:
                        if traci.vehicle.isStopped(ith_robot):
                            stop_node = int(last_edge[last_edge.find('to') + 2:])
                            print("Stop node: "+ str(stop_node))
                            srvs.node_id.append(str(stop_node))
                            srvs.robot_id.append(ith_robot)
                            # self.last_nodes[i] = stop_node
                            self.stopped_already[ith_robot] = True

                        elif not traci.vehicle.isStopped(ith_robot) and self.last_nodes[ith_robot] != last_node:
                            total_len = traci.lane.getLength(last_edge + '_0')
                            cur_len = traci.vehicle.getLanePosition(ith_robot)
                            if cur_len < total_len/2:
                                stop_pos = traci.lane.getLength(self.seq_of_routes[ith_robot][-1] + '_0')
                                traci.vehicle.setStop(vehID = ith_robot, edgeID = self.seq_of_routes[ith_robot][-1], pos = stop_pos, duration = 2000.)
                                #msg.node_id.append(int(last_node))
                                srvs.node_id.append(str(last_node))
                                srvs.robot_id.append(ith_robot)
                                self.last_nodes[ith_robot] = last_node
            
            
            if len(srvs.node_id) > 0:
                rospy.loginfo_throttle(1,'Task done node_ids are '+ str(srvs.node_id))
                print(self.sumo_to_pmm_proxy(srvs.stamp, srvs.node_id, srvs.robot_id))
                for vehicle_in_sumo in traci.vehicle.getIDList():
                    print(vehicle_in_sumo)
                    if vehicle_in_sumo not in self.robot_ids:
                        traci.vehicle.remove(vehicle_in_sumo, 0x03)
                        del self.seq_of_routes[vehicle_in_sumo]
                        del self.updated[vehicle_in_sumo]
                        del self.last_nodes[vehicle_in_sumo]
                        del self.stopped_already[vehicle_in_sumo]
                        self.num_vehicles = self.num_vehicles - 1

            # print 'Num_vehicles', traci.vehicle.getIDCount()
            #print (t.stopped_already.values())               
            #print( sum(t.stopped_already.values()))
            

            if sum(self.stopped_already.values()) == 0:
                traci.simulationStep()

        rospy.logwarn('simulation startup exiting')
        rospy.set_param('done', True)
        traci.close()


    def callback_init(self, init_msg):
        rospy.loginfo("Callback for initialization of TraCI called.")
        global flag_already_initialized
        if flag_already_initialized:
            rospy.logwarn_throttle(1," Callback for initialization for TraCI called again. Initialization request shall be sent only once.")    
        else:
            flag_already_initialized = True
            self.dirname = rospkg.RosPack().get_path('patrol_algo')
            self.num_vehicles = init_msg.num_robots
            self.min_time = init_msg.min_time
            self.graph_name = init_msg.graph
            self.robot_ids = init_msg.robot_ids.split(' ')
            #self.robot_ids = []
            rospy.loginfo("%s, %s, %s, %s, robot_ids %s",self.dirname, self.num_vehicles, self.min_time, self.graph_name, str(self.robot_ids))
            #simulation_startup(dirname, num_vehicles, min_time, graph_name, robot_ids)
            self.simulation_startup()

# def valid_walks_done_CB(msg):
#     if msg.valid_walks_done:
#         print("SUMO Received Start confirmation")
#         s = Sumo_Sim()   

if __name__ == '__main__':
    rospy.init_node('sumo_sim') 
    rospy.loginfo("Main started")
    # valid_walks_done_sumo = rospy.Subscriber('valid_walks_topic', ValidWalksDone, valid_walks_done_CB)
    s = Sumo_Sim()   
    rospy.spin()
    rospy.loginfo("Main ended")
    
 

