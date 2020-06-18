#!/usr/bin/env python

'''
Online module of TPBP Algo on walks
'''
import rospy
import rospkg
import pdb
import os
import networkx as nx
from patrol_messages.msg import *
from patrol_messages.srv import *
from sets import Set
import networkx as nx
import numpy
import time


def tpbp_reward(g, algo_params, walk, priority_nodes, time_periods, assigned):
    nodes = list(Set(walk))
    term1 = 0.
    term2 = 0.
    for i in nodes:
        term1 += g.node[i]['idleness']
        if i in priority_nodes:
            j = priority_nodes.index(i)
            if not assigned[j]:
                term2 += max(g.node[i]['idleness'] - time_periods[j], 0)
    term3 = 0.
    for i in range(len(priority_nodes)):
        if not assigned[i] and not priority_nodes[i] in nodes:
            dist = numpy.inf
            for j in nodes:
                temp = nx.shortest_path_length(g, j, priority_nodes[i])
                if temp < dist:
                    dist = temp
            term3 += dist
    term4 = 0.
    for i in range(len(walk) - 1):
        term4 += g[walk[i]][walk[i + 1]]['length']
    return numpy.dot([term1, term2, term3, term4], algo_params)

def tpbp_walk(road_graph, algo_params, cur_node, priority_nodes, time_periods, assigned, folder_path):
    best_reward = -numpy.inf
    best_walk = []
    rospy.loginfo(priority_nodes)
    rospy.loginfo(assigned)
    for j in range(len(priority_nodes)):
        if not assigned[j]:
            valid_trails = '/valid_trails_{}_{}_{}.in'.format(cur_node, priority_nodes[j], str(int(time_periods[priority_nodes.index(cur_node)])))
            try:
                with open(folder_path + valid_trails, 'r') as f:
                    count = 0
                    for line in f:
                        count += 1
                        line1 = line.split('\n')
                        line2 = line1[0].split(' ')
                        r = tpbp_reward(road_graph, algo_params, line2, priority_nodes, time_periods, assigned)
                        if r > best_reward:
                            best_reward = r
                            best_walk = line2
            except Exception,e:
                rospy.logerr(" Exception in onpening valid trail file %s",valid_trails)
                rospy.logerr(e)
        else:
            print str(j)+' assigned '
    print 'Final Selected best walk is ' + str( best_walk)    
    return best_walk

class TPBP:
    def __init__(self):
        self.is_initial_params_received = False
        self.initial_parm_subs = rospy.Subscriber('pmm_to_algo_params', Initialize_patrol, self.initializaiton_params_CB)
        self.team_update_to_tpbp_service = rospy.Service('team_update_to_tpbp', TeamUpdateRobotSrv, self.team_update_CB)
        self.timer = rospy.Timer(rospy.Duration(1),self.timed_algo_setup_cb, oneshot=True)
        
        rospy.loginfo("TPBP Constructor done!!!")

    def timed_algo_setup_cb(self, timer):
        rospy.logdebug_throttle(1,"timer callback")
        self.algorithm_setup()

    def team_update_CB(self, request):      
        rospy.wait_for_service('team_update_to_sumo')
        self.team_update_to_sumo_proxy = rospy.ServiceProxy('team_update_to_sumo', TeamUpdateRobotSrv)
        try:
            print(self.team_update_to_sumo_proxy(request.robot_id_list, request.robot_id))
        except Exception as e:
            rospy.logerr(e)

        if(len(self.robot_ids) < len(request.robot_id_list)):           #if a robot is added
            indx = 0
            self.robot_ids = request.robot_id_list 
            print("Robot ids after addition: "+ str(self.robot_ids))            
            rospy.loginfo("Computing initial tpbp_walk for robot %s", request.robot_id)
            for i in self.assigned:
                if self.assigned[i] == False:
                    indx = i
            self.robot_cur_walks[request.robot_id] = tpbp_walk(self.graph, self.algo_params, self.priority_nodes[indx], self.priority_nodes, self.time_periods, self.assigned, self.dest_folder)
            rospy.loginfo('PRINTING Dict')
            rospy.loginfo(self.robot_cur_walks)
            self.assigned[self.priority_nodes.index(self.robot_cur_walks[request.robot_id][-1])] = True
            self.task_list[request.robot_id] = map(str, self.robot_cur_walks[request.robot_id])
            rospy.loginfo(self.task_list) 
            print("SERVICE MESSAGE")
            print(self.tpbp_to_pmm_proxy(map(str, self.robot_cur_walks[request.robot_id]), request.robot_id))

        elif(len(self.robot_ids) > len(request.robot_id_list)):          #if a robot is removed
            self.robot_ids = request.robot_id_list 
            print("Robot ids after removal: "+ str(self.robot_ids)) 
            self.assigned[self.priority_nodes.index(self.robot_cur_walks[request.robot_id][-1])] = False
            del self.robot_cur_walks[request.robot_id]
            del self.task_list[request.robot_id]
        
        return TeamUpdateRobotSrvResponse("Team Update to TPBP received")

    #     #to_update_robot_id = team_update_msg.robot_id
    #     rospy.sleep(2)
    #     for robot_opr in team_update_msg.robot_operations:
    #         robot_id = robot_opr.robot_id
    #         opr = robot_opr.operation
    #         print("Robot id: " + str(robot_id) + " added")
    #         if opr == robot_operation_assignment.ADD_TO_PATROL:
    #             self.add_robot_id_to_team(robot_id)
    #         elif opr == robot_operation_assignment.REMOVE_FROM_PATROL:
    #             self.remove_robot_id_from_team(robot_id)
    #         else:
    #             rospy.logerr('Unknown robot team operations specified')

    def algorithm_setup(self):
        rospy.sleep(2)
        rospy.loginfo("Algorithm setup initiated: TPBP ")
        rospy.loginfo("%s, %s, %s, %s, %s, %s, %s, %s",self.dirname, self.folder, self.num_robots, self.priority_nodes, self.graph_name, self.algo, self.algo_params, self.time_periods)
        print("Inside algo_setup Robot ids: "+ str(self.robot_ids))
        if self.algo != "tpbp_walk":
            rospy.logwarn(" Algorithm is not TPBP_WALK. Received algo is <%s>", self.algo)
            return
        self.dest_folder = self.dirname + '/processing/' + self.folder
        self.graph = nx.read_graphml(self.dirname + '/graph_ml/' + self.graph_name + '.graphml')
        for node in self.graph.nodes():
            self.graph.node[node]['idleness'] = 0.
            rospy.loginfo_throttle(1, ' node idleness set to 0')
        self.stamp = 0.0
        self.assigned = []
        for i in self.priority_nodes:
            self.assigned.append(False)
        self.robot_cur_walks = {}
        self.task_list = {}
        
        #self.task_done_subs = rospy.Subscriber('/task_done', TaskDone, self.task_done_CB)
        self.tpbp_service = rospy.Service('pmm_to_tpbp', TaskDoneSrv, self.task_done_CB)
        rospy.logdebug(" Constructing TPBP _line_=91 ")       
        for index, ith_robot in zip(range(len(self.robot_ids)), self.robot_ids):
            rospy.loginfo("Computing initial tpbp_walk for robot %s", ith_robot)
            time_start = time.clock()
            self.robot_cur_walks[ith_robot] = tpbp_walk(self.graph, self.algo_params, self.priority_nodes[index], self.priority_nodes, self.time_periods, self.assigned, self.dest_folder)
            print(self.robot_cur_walks)
            rospy.loginfo('PRINTING Dict')
            rospy.loginfo(self.robot_cur_walks)
            self.assigned[self.priority_nodes.index(self.robot_cur_walks[ith_robot][-1])] = True
            time_end =time.clock()
            time_spend = time_end - time_start
            rospy.loginfo(" Completed computation of tpbp_walk. Time spent %f", time_spend)                        
            #self.task_list.append(map(int, self.robot_cur_walks[i]))
            rospy.loginfo("TASK_LIST PRINT:")
            self.task_list[ith_robot] = map(str, self.robot_cur_walks[ith_robot])
            rospy.loginfo(self.task_list) 
        rospy.loginfo(self.robot_cur_walks) 
        rospy.loginfo("Line _163_ Robot ids: %s", str(self.robot_ids))
        for ith_robot in self.robot_ids:
            rospy.loginfo("inside for of robot id %s", ith_robot)

            try:
                rospy.wait_for_service('tpbp_to_pmm')
                self.tpbp_to_pmm_proxy = rospy.ServiceProxy('tpbp_to_pmm', NextTaskSrv)
                try:
                    print(self.tpbp_to_pmm_proxy(self.task_list[ith_robot], ith_robot))
                    rospy.loginfo(self.task_list[ith_robot])
                    rospy.loginfo(ith_robot)
                except Exception,e:
                    rospy.logwarn(" Not able to publish the next task for robot %s Exception is:", ith_robot)
                    rospy.logerr(e)            
            except Exception,e:
                rospy.logerr(e)
                rospy.logerr("unable to create service client for robot %s for topic %s", ith_robot)

        rospy.loginfo("TPBP Setup done!!!")

    def __del__(self):
        rospy.logwarn(" TPBP destructor called ")
        print("TPBP destructor called")
    
    def task_done_CB(self, request):
        rospy.loginfo_throttle(1,"ENTERED /task_done CALLBACK. ")
        print ('Received Task Done message is: ')
        print (request)

        if self.stamp <= request.stamp:# to ignore out-of-order messages from the simulator
            #Update Idleness
            dev = request.stamp - self.stamp
            self.stamp = request.stamp
            for i in self.graph.nodes():
                self.graph.node[i]['idleness'] += dev
                #print(self.graph.node[i]['idleness'])
            for i in range(len(request.node_id)):
                    self.graph.node[str(request.node_id[i])]['idleness'] = 0
                    #print(self.graph.node[str(task_done_msg.node_id[i])]['idleness'])
            
            #Update Walk If Necessary            
            for i, itr_robot_id in zip(range(len(request.robot_id)),request.robot_id):
                print_msg = "Checking for updates to walk of robot " + itr_robot_id
                print(print_msg)                
                n = request.node_id[i]
                m = self.robot_cur_walks[request.robot_id[i]][0]
                print str(n), m
                if str(n) == m:  
                    print("robot_cur_walks b4 " , self.robot_cur_walks[request.robot_id[i]])                   
                    self.robot_cur_walks[request.robot_id[i]].pop(0)    
                    print("robot_cur_walks after " , self.robot_cur_walks[request.robot_id[i]])        

                if len(self.robot_cur_walks[request.robot_id[i]]) == 0: 
                    self.assigned[self.priority_nodes.index(str(n))] = False
                    self.robot_cur_walks[itr_robot_id] = tpbp_walk(self.graph, self.algo_params, str(n), self.priority_nodes, self.time_periods, self.assigned, self.dest_folder)
                    rospy.loginfo('PRINTING Dict')
                    rospy.loginfo(self.robot_cur_walks)
                    self.assigned[self.priority_nodes.index(self.robot_cur_walks[itr_robot_id][-1])] = True
                    print("SERVICE MESSAGE")
                    print(self.tpbp_to_pmm_proxy(map(str, self.robot_cur_walks[itr_robot_id]), itr_robot_id))            
                
        else:
            rospy.logwarn_throttle(1, ' self.stamp is NOT less than request.stamp !')
        return TaskDoneSrvResponse("Task Done Params Received by TPBP!!!")
  
      
    def initializaiton_params_CB(self, init_msg):
        self.dirname = rospkg.RosPack().get_path('patrol_algo')
        self.folder = init_msg.folder
        self.num_robots = init_msg.num_robots
        self.priority_nodes = init_msg.priority_nodes.split(' ')
        self.graph_name = init_msg.graph
        self.algo = init_msg.algo
        self.algo_params = map(float, init_msg.algo_params.split(' '))
        self.time_periods = map(float, init_msg.time_periods.split(' '))
        self.robot_ids = init_msg.robot_ids.split(' ')
        self.initial_parm_subs.unregister()
        rospy.loginfo("Received Initial params: %s, %s, %s, %s, %s, %s, %s, %s, %s", self.dirname, self.folder, self.num_robots, self.priority_nodes, self.graph_name, self.algo, self.algo_params, self.time_periods, self.robot_ids)    

# def valid_walks_done_CB(msg):
#     if msg.valid_walks_done:
#         print("TPBP_ONLINE Received Start confirmation")
#         t = TPBP()   
#         done = False
#         del t
    
if __name__ == '__main__':
    rospy.init_node('tpbp_online', anonymous = False)   
    # valid_walks_done_tpbp = rospy.Subscriber('valid_walks_topic', ValidWalksDone, valid_walks_done_CB)
    t = TPBP()   
    rospy.spin()
    done = False
    # # while not done:
    # #     done = rospy.get_param('/done')
    del t
    