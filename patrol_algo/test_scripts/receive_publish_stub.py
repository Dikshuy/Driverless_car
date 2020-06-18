#!/usr/bin/env python

'''
Online module of TPBP Algo on walks
'''
import rospy
import rospkg
import os
import networkx as nx
#import ConfigParser as CP
from patrol_messages.msg import *
from sets import Set
import numpy
import time

class DummyPublisherSubscriber:
    def __init__(self):
        self.is_initial_params_received = False
        #self.initial_parm_subs = rospy.Subscriber('pmm_to_algo_params', Initialize_patrol, self.initializaiton_params_CB)
        self.subs = rospy.Subscriber('/robot_0/next_task', NextTask, self.read_msg_cb)
        self.timer = rospy.Timer(rospy.Duration(1),self.timed_algo_setup_cb, oneshot=True)        
        self.robot_cur_walks = {}
        self.list_robot_pub = []
        rospy.loginfo("TPBP Constructor done!!!")

    def timed_algo_setup_cb(self, timer):
        rospy.logdebug_throttle(1,"timer callback")
        self.algorithm_setup()

    def read_msg_cb(self, next_task_msg):
        rospy.loginfo("\n Received task msg is: ")
        rospy.loginfo(next_task_msg)
        rospy.loginfo("\n")

    def algorithm_setup(self):
        #rate = rospy.Rate(10) # 10hz
        rospy.loginfo("Algorithm setup initiated: TPBP ")        
        rospy.logdebug(" Constructing TPBP _line_=91 ")       
        for i in range(1):
            rospy.loginfo("Computing initial tpbp_walk for robot %d", i)
            time_start = time.clock()
            self.robot_cur_walks[i] = ['0','0','0']        
        for i in range(1):
            #print i
            rospy.loginfo("robot %d",i)
            topic_name = '/robot_{}/next_task'.format(i)
            rospy.loginfo("creating the publisher for robot %d on topic %s", i,topic_name)
            try:                
                robot_i_next_task_publisher = rospy.Publisher('/robot_{}/next_task'.format(i), NextTask, latch=True, queue_size = 10)       #latch is important. self.subs was unable to read without it. Publisher was able to publish, before the queues for topics are set.      
                #print(robot_i_next_task_publisher)
                next_task_msg = NextTask()
                next_task_msg.task =  self.robot_cur_walks [i]                
                try:
                    robot_i_next_task_publisher.publish(next_task_msg)
                    rospy.loginfo(" next task published for robot %d is :",i)
                    rospy.loginfo(next_task_msg)
                except Exception,e:
                    rospy.logwarn(" Not able to publish the next task for robot %d on topic %. Exception is:", i, topic_name)
                    rospy.logerr(e)
                try:            
                    self.list_robot_pub.append(robot_i_next_task_publisher)                
                except Exception:
                    rospy.logwarn(" Unable to insert the robot publisher inside the list")
            except Exception, e:
                rospy.logwarn(" unable to create publisher for robot %d for topic %s", i, '/robot_{}/next_task'.format(i))
                rospy.logwarn(e)
            
        rospy.loginfo("TPBP Setup done!!!")

    def __del__(self):
        rospy.logwarn(" TPBP destructor called ")
        print("TPBP destructor called")
    
    # def initializaiton_params_CB(self, init_msg):
    #     self.dirname = rospkg.RosPack().get_path('patrol_algo')
    #     self.folder = init_msg.folder
    #     self.num_vehicles = init_msg.num_robots
    #     self.priority_nodes = init_msg.priority_nodes.split(' ')
    #     self.graph_name = init_msg.graph
    #     self.algo = init_msg.algo
    #     self.algo_params = map(float, init_msg.algo_params.split(' '))
    #     self.time_periods = map(float, init_msg.time_periods.split(' '))
    #     rospy.loginfo("Received Initial params: %s, %s, %s, %s, %s, %s, %s, %s",self.dirname, self.folder, self.num_vehicles, self.priority_nodes, self.graph_name, self.algo, self.algo_params, self.time_periods)
        
if __name__ == '__main__':
    rospy.init_node('tpbp_online', anonymous = False)     
    t = DummyPublisherSubscriber()
    rospy.spin()
    done = False
    # while not done:
    #     done = rospy.get_param('/done')
    del t
    