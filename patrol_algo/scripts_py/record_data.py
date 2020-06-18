#!/usr/bin/env python

import rospy
import rospkg
import os, sys
#import ConfigParser as CP
from patrol_messages.msg import *
import networkx as nx
import pandas as pd



class Record_Data:

    def __init__(self, dirname, folder, graph_name, num_vehicles, min_time):
        rospy.init_node('record_data', anonymous = True)
        self.dirname = dirname
        self.folder = folder
        self.dest_file = self.dirname + '/post_processing/' + self.folder + '/sim_data.in'
        self.dest_folder = self.dirname + '/post_processing/' + self.folder
        rospy.Subscriber('/task_done', TaskDone, self.callback)
        self.stamp = -1.
        self.graph_name = graph_name
        self.num_robots = num_vehicles
        self.graph = nx.read_graphml(self.dirname + '/graph_ml/' + self.graph_name + '.graphml')
        self.total_time = min_time

    def callback(self, data):
        if self.stamp < data.stamp:
            self.stamp = data.stamp
            with open(self.dest_file, 'a+') as f:
                f.write(str(data.stamp) + '\n')
                f.write(' '.join(map(str, data.node_id)) + '\n')
                f.write(' '.join(map(str, data.robot_id)) + '\n')

    def post_process(self):
        count = 0        
        nodes = self.graph.nodes()
        node_data = {}
        for i in nodes:
            node_data[i] = []        
        time_stamps = []
        robot_data = {}
        for i in range(self.num_robots):
            robot_data[i] = [] 
        # node_data = pd.DataFrame()
        nod = []
        rob = []
        time = 0
        with open(self.dest_file, 'a+') as f:
            for line in f:
                if count % 3 == 0:
                    time = int(float(line.strip('\n')))
                    time_stamps.append(time)
                elif count % 3 == 1:
                    l = line.strip('\n')
                    nod = l.split(' ')
                    for i in nod:
                        node_data[i].append(time)
                else:
                    l = line.strip('\n')
                    rob = map(int, l.split(' '))
                    for i in range(len(rob)):
                        robot_data[rob[i]].append((time, nod[i]))
                count += 1

        for i in range(self.num_robots):
            rob_file = self.dest_folder + '/robot_{}.in'.format(i)
            df = pd.DataFrame(robot_data[i])
            df.to_csv(rob_file, sep = ' ', index = False, header = False)
        
        node_data2 = {}
        for i in nodes:
            node_data2[i] = [0]
            for j in range(1, self.total_time + 1):
                if j in node_data[i]:
                    node_data2[i].append(0)
                else:
                    node_data2[i].append(node_data2[i][-1] + 1)
        
        nod_file = self.dest_folder + '/node_idleness.csv'
        df = pd.DataFrame(node_data2)
        df.to_csv(nod_file)

def callback_init(init_msg):
    dirname = rospkg.RosPack().get_path('patrol_algo')
    folder = init_msg.folder
    num_vehicles = init_msg.num_robots
    graph_name = init_msg.graph
    min_time = init_msg.min_time
    t = Record_Data(dirname, folder, graph_name, num_vehicles, min_time)
    done = False
    while not done:
        try:
            done = rospy.get_param('/done')
        except KeyError as e:
            print("done still not present ...")                
	t.post_process()
    sub.unregister()

if __name__ == '__main__':
    sub = rospy.Subscriber('pmm_to_algo_params', Initialize_patrol, callback_init)
    
  
