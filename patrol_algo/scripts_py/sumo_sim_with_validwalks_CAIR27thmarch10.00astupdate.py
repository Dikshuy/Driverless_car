#!/usr/bin/env python

import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib
import pdb
import rospy, rospkg
from enum import Enum
from multiprocessing import Pool
from random import random
import networkx as nx
from patrol_messages.msg import *
from patrol_messages.srv import *
import math
from collections import OrderedDict

flag_already_initialized = False

color_list= ["red", "green", "blue", "yellow", "orange"]
color_dict = { 0:[1,0,0], 1:[0,1,0], 2:[0,0,1,1], 3:[1,0,1,1], 4:[0,1,1,1]}

robot_string_2_robot_num = {}
robot_num_2_robot_string = {}
max_robot_numeral_id = 0
HIGH_NUM = '5000'

def inc_count (a_dict, comp_key):
    for key, value in a_dict.items():
        if key == comp_key:         
            value += 1
            a_dict[key] = value

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
        self.heat_map_key = []
        self.cair_data_dict =  OrderedDict()
        self.cair_graph = nx.read_graphml(rospkg.RosPack().get_path('patrol_algo')+'/graph_ml/{}.graphml'.format(self.graph_name))
        self.cair_data_dict = self.cair_graph.edge
        for key, item in self.cair_data_dict.items():
            for r in item:
                self.heat_map_key.append(item[r]['name'])
        #print "here1........", self.heat_map_key, '\n', len(self.heat_map_key)
        self.team_update_to_sumo_service = rospy.Service('team_update_to_sumo', TeamUpdateRobotSrv, self.team_update_CB)
        self.sumo_service = rospy.Service('pmm_to_sumo', NextTaskSrv, self.pmm_sumo)
        self.rem_after_chk = False
        self.robot_remove_after_walk = ''
        self.prev_last_edge = {}

        for ith_robot in self.robot_ids:
            self.seq_of_routes[ith_robot] = []
            self.prev_last_edge[ith_robot]=''
            self.updated[ith_robot] = False
            self.last_nodes[ith_robot] = HIGH_NUM
            self.stopped_already[ith_robot] = True

        #self.pub = rospy.Publisher('/task_done', TaskDone, queue_size = 10)
        rospy.wait_for_service('sumo_to_pmm')
        self.sumo_to_pmm_proxy = rospy.ServiceProxy('sumo_to_pmm', TaskDoneSrv)

        rospy.loginfo("TraciSim constructon completed")

    def calcdist(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def maxdist(self, a, bList=[]):
        max_dist = 0
        for r in range(len(bList)):
        	dist = self.calcdist(a, bList[r])
        	#print dist
        	if dist > max_dist:
        	   max_dist = dist
        return max_dist

    def pmm_sumo(self, request):
        print('\n\nNEXT TASK RECEIVED FROM PMM CALLED\n\n')
        print(request.task, request.robot_id)
        
        route = []
        for i in range(len(request.task) - 1):
            route.append(self.cair_graph.edge[request.task[i]][request.task[i + 1]]['name'])
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
            self.prev_last_edge[ith_robot] = ''
            self.updated[request.robot_id] = False
            self.last_nodes[request.robot_id] = HIGH_NUM
            self.stopped_already[request.robot_id] = True
            self.num_vehicles = self.num_vehicles + 1

        elif(len(self.robot_ids) > len(request.robot_id_list)):     #if a robot is removed
            self.robot_ids = request.robot_id_list
            self.robot_remove_after_walk = request.robot_id 
            self.rem_after_chk = request.rem_check
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


    def simulation_startup(self, prior_nodes):
        rospy.loginfo("Entered simulation_startup. num_vehicles: %d" ,self.num_vehicles)
        rospy.set_param('done', False)
        #self.prior_nodes = ['1563324344', 'Custom21', 'Custom_fix_1', '1563324004']

        sumo_startup = ['sumo-gui', '-c', self.dirname +'/graph_sumo/{}.sumocfg'.format(self.graph_name)]


        rospy.loginfo(sumo_startup)
        # print 'Click Play button only on ensuring that the patrolling algorithm is running'
        traci.start(sumo_startup)

        net = sumolib.net.readNet(self.dirname +'/graph_sumo/{}.net.xml'.format(self.graph_name))
        print "priority nodes: ", len(prior_nodes), prior_nodes

        count = 0
        for p_node in prior_nodes:
            radius = (self.maxdist(net.getNode(p_node).getCoord(), net.getNode(p_node).getShape()) * 1.2)
            center = net.getNode(p_node).getCoord()
            shape_val = []
            for r in range (0, 370, 30):
                #print r, math.radians(r), center[0], radius
                x = center[0] + radius * (math.cos(math.radians(r)))
                y = center[1] + radius * (math.sin(math.radians(r)))
                shape_val.append((x,y))
            #print "here---------", p_node, shape_val

	    traci.polygon.add(polygonID=str(count), shape=shape_val, color=(255,0,0), fill=True, polygonType='', layer=0, lineWidth=2)
	    count +=1

        rospy.loginfo("Simulation Startup Occured")
        #Click Play Button in the GUI, if GUI
        # pool = Pool(1)                  #CHECKING BATTERY LEVELS
        rospy.sleep(40)
        # p1 = pool.apply_async(self.battery_check_func())
        # print("Thread started")

        while traci.simulation.getTime() <= self.min_time:
            #print traci.simulation.getTime()
            vehicles = traci.vehicle.getIDList()
            #rospy.loginfo("VEHICLES : %s", str(vehicles))
            for itr_robot_i in self.robot_ids:
                if self.updated[itr_robot_i]:
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
                #print("\n******************************** Seq:  " + itr_robot_i + "--" + str(self.seq_of_routes[itr_robot_i]))
                self.updated[itr_robot_i] = False

            srvs = TaskDoneSrv()
            srvs.stamp = traci.simulation.getTime()
            srvs.node_id = []
            srvs.robot_id = []
            last_edge_update = {}
            for ith_robot in self.robot_ids:
                vehicles = traci.vehicle.getIDList()
                if ith_robot in vehicles:
                    last_edge = traci.vehicle.getRoadID(ith_robot)
                    last_edge_update[ith_robot] = last_edge
                    print ("Robot_id....", ith_robot, self.seq_of_routes[ith_robot])
                    if last_edge in self.heat_map_key:
                        for key, value in self.prev_last_edge.items():
                            print "here6...", key, value         
                        if ith_robot in self.prev_last_edge.keys():
                            print "here7", self.prev_last_edge[ith_robot], self.prev_last_edge[str(ith_robot)], self.prev_last_edge.get(ith_robot), 'nex', self.prev_last_edge.get(str(ith_robot)), self.prev_last_edge.get(ith_robot,None)

                        else:
                            print ith_robot, "not found in the list"
                        print ("Last edge: " , ith_robot, str(last_edge), str(last_edge_update[ith_robot]),self.prev_last_edge[ith_robot], str(last_edge) == str(self.prev_last_edge[str(ith_robot)]))
                    #else:
                    #    print ("Last edge:.......... " , str(last_edge), str(self.prev_last_edge))
                    #print(type(last_edge))
                    if last_edge.find(':') == -1:
                        for ed in self.cair_graph.edges():
                            if last_edge == self.cair_graph.edge[ed[0]][ed[1]]['name']:
                                last_node = ed[0]
                                stop_node = ed[1]
                        print ("Last node: " + str(last_node) , str (stop_node))
                        # if traci.vehicle.isStopped(str(i)) and not t.stopped_already[i]:
                        if traci.vehicle.isStopped(ith_robot):
                            print("Stop node: "+ str(stop_node))
                            srvs.node_id.append(str(stop_node))
                            srvs.robot_id.append(ith_robot)
                            # self.last_nodes[i] = stop_node
                            self.stopped_already[ith_robot] = True

                        elif not traci.vehicle.isStopped(ith_robot) and self.last_nodes[ith_robot] != last_node:
                            total_len = traci.lane.getLength(last_edge + '_0')
                            cur_len = traci.vehicle.getLanePosition(ith_robot)
                            if cur_len < 0.9*total_len:
                                stop_pos = traci.lane.getLength(self.seq_of_routes[ith_robot][-1] + '_0')
                                traci.vehicle.setStop(vehID = ith_robot, edgeID = self.seq_of_routes[ith_robot][-1], pos = stop_pos, duration = 2000.)
                                #msg.node_id.append(int(last_node))
                                srvs.node_id.append(str(last_node))
                                srvs.robot_id.append(ith_robot)
                                self.last_nodes[ith_robot] = last_node


            if len(srvs.node_id) > 0:
                rospy.loginfo_throttle(1,'Task done node_ids are '+ str(srvs.node_id))  
                print "here5", srvs.robot_id   
                self.prev_last_edge[str(srvs.robot_id[0])] = last_edge_update[str(srvs.robot_id[0])]
                for key, value in self.prev_last_edge.items():
                    print "here5...", key, value         
                print(self.sumo_to_pmm_proxy(srvs.stamp, srvs.node_id, srvs.robot_id))
                for vehicle_in_sumo in traci.vehicle.getIDList():
                    if vehicle_in_sumo not in self.robot_ids:                        
                        if vehicle_in_sumo == self.robot_remove_after_walk and self.rem_after_chk == True:
                            #print 'here1:', vehicle_in_sumo, traci.vehicle.getRoadID(vehicle_in_sumo), traci.vehicle.getRoute(vehicle_in_sumo)[-1]
                            if traci.vehicle.isStopped(vehicle_in_sumo):
                                #print "here2", vehicle_in_sumo, "is stopped"
                                traci.vehicle.remove(vehicle_in_sumo, 0x03)
                                del self.seq_of_routes[vehicle_in_sumo]
                                del self.updated[vehicle_in_sumo]
                                del self.last_nodes[vehicle_in_sumo]
                                del self.stopped_already[vehicle_in_sumo]
                                self.num_vehicles = self.num_vehicles - 1
                        else:
                            print "In sumo:", vehicle_in_sumo, traci.vehicle.getRoadID(vehicle_in_sumo)
                            traci.vehicle.remove(vehicle_in_sumo, 0x03)
                            del self.seq_of_routes[vehicle_in_sumo]
                            del self.updated[vehicle_in_sumo]
                            del self.last_nodes[vehicle_in_sumo]
                            del self.stopped_already[vehicle_in_sumo]
                            self.num_vehicles = self.num_vehicles - 1

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
            prior_nodes = init_msg.priority_nodes.split(' ')

            #self.robot_ids = []
            rospy.loginfo("%s, %s, %s, %s, robot_ids %s %s",self.dirname, self.num_vehicles, self.min_time, self.graph_name, str(self.robot_ids), str(prior_nodes))
            #simulation_startup(dirname, num_vehicles, min_time, graph_name, robot_ids)
            self.simulation_startup(prior_nodes)

def valid_walks_done_CB(msg):
    if msg.valid_walks_done:
        print("SUMO Received Start confirmation")
        s = Sumo_Sim()

if __name__ == '__main__':
    rospy.init_node('sumo_sim')
    rospy.loginfo("Main started")
    valid_walks_done_sumo = rospy.Subscriber('valid_walks_topic', ValidWalksDone, valid_walks_done_CB)
    # s = Sumo_Sim()
    rospy.spin()
    rospy.loginfo("Main ended")
