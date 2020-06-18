from controller import Supervisor
from collections import defaultdict
import re
import numpy as np
import math
import rospy
from patrol_messages.msg import *
from time import sleep
TIME_STEP = 32

class AddRemoveBots(object):
    def __init__(self):
        #rospy.init_node('Robot_operation_module', anonymous = False)
        self.rob_update_subs = rospy.Subscriber('add_remove_bots', rob_webots, self.update_CB)
        self._msg = None
        self.prev_msg = None

    def update_CB(self, rob_webots_msg):
        self._msg = rob_webots_msg

    def get_msg(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        if self._msg != None and self._msg != self.prev_msg:
            #self._event.wait(timeout)
            self.prev_msg = self._msg
            return self._msg

class EgoVehicle:
    def __init__(self):
        self.robot_ids = []
        self.supervisor = Supervisor()
        self.rootChildren = self.supervisor.getRoot().getField("children")
        self.count = 0

        with open ('/home/dikshant/catkin_ws/src/webots_ros/worlds/CAIR_mod_net/sumo.net.xml') as f:
            for line in f:
                if "location netOffset=" in line:
                    word = line.split(' ')
                    for word_l in word:
                        if 'netOffset=' in word_l:
                            offset_value = re.findall("\d+\.\d+", word_l)
        f.close()
        self.xOffset = float(offset_value[0])
        self.yOffset = float(offset_value[1])
        #print (xOffset, yOffset)

        self.edge_dict = defaultdict(dict)
        #Read values from a file corresponding to the edge
        with open ('/home/dikshant/catkin_ws/src/webots_ros/worlds/CAIR_mod_net/CAIR_mod_edge_info.in') as f1:
            for line in f1:
                coord = line.split()
                if len(coord) == 1:
                    key_value = coord[0]
                    x_co = []
                    y_co = []
                elif len(coord) == 2:
                    #print coord
                    #Each x added with xOffset
                    x_co.append(float(coord[0]) + self.xOffset)
                    #Each y added with yOffset
                    y_co.append(float(coord[1]) + self.yOffset)
                else:
                    self.edge_dict[key_value]['x_values'] = x_co
                    self.edge_dict[key_value]['y_values'] = y_co
        self.run()

    def run (self):
        rospy.init_node('sumo_sim', anonymous=True)
        self.rob_oper = AddRemoveBots()
        self.robot_ids = ["r0", "r1", "r2", "r3"]
        self.start_edge_ids = ['142865550', '142865547_2', '99829041_2', '-142865547_2']
        # Get initial vehicles in simulation
        for i in range(len(self.robot_ids)):
            #Generate Vehicle
            defName = "EGO_VEHICLE_%s" % self.robot_ids[i]
            vehicleString = "DEF " + defName + " " + "TeslaModel3" + " {\n"
            x_coord = []
            y_coord = []
            xp, yp, zp = [], [], []
            key_val = self.start_edge_ids[i]
            x_coord = self.edge_dict[key_val]['x_values']
            y_coord = self.edge_dict[key_val]['y_values']
            height = 0.4

            for index in range(2):
                pos = [round(-x_coord[index]  + self.xOffset , 2), height, y_coord[index] - self.yOffset]
                zp.append(pos[2])
                xp.append(pos[0])

            #Finding rotation and translation coordinates
            x1 = xp[0]
            x2 = xp[1]

            z1 = zp[0]
            z2 = zp[1]
            vehicleString += "  translation {} {} {}\n".format(x1, height, z1)
            # angle
            ang_radians = math.atan2(x2-x1, z2-z1)
            vehicleString += "  rotation 0 1 0 {}\n".format(ang_radians)
            vehicleString += "  color 0.18 0.50 0.72\n"
            vehicleString += "  name \"{}\"\n".format(self.robot_ids[i])
            vehicleString += "  controller \"{}\"\n".format("ros_automobile")
            vehicleString += "  controllerArgs \"--name={} --clock --use-sim-time\"\n".format(self.robot_ids[i])
            vehicleString += "  sensorsSlotCenter [\n"
            vehicleString += "    GPS {\n"
            vehicleString += "    }\n"
            vehicleString += "  ]\n"
            vehicleString += "  }\n"
            self.rootChildren.importMFNodeFromString(-1, vehicleString)


        self.count = i+1

        while self.supervisor.step(TIME_STEP) != -1:
            print(self.robot_ids)
            msg = self.rob_oper.get_msg()
            if msg != None:
                if msg.opr == 'add':
                    if msg.rob_id not in self.robot_ids:
                        defName = "EGO_VEHICLE_%s" % msg.rob_id
                        vehicleString = "DEF " + defName + " " + "TeslaModel3" + " {\n"
                        x_coord = []
                        y_coord = []
                        xp, yp, zp = [], [], []
                        key_val = msg.edge_id
                        x_coord = self.edge_dict[key_val]['x_values']
                        y_coord = self.edge_dict[key_val]['y_values']
                        height = 0.4

                        for ind in range(2):
                            pos = [round(-x_coord[ind]  + self.xOffset , 2), height, y_coord[ind] - self.yOffset]
                            zp.append(pos[2])
                            xp.append(pos[0])

                        #Finding rotation and translation coordinates
                        x1 = xp[0]
                        x2 = xp[1]

                        z1 = zp[0]
                        z2 = zp[1]
                        vehicleString += "  translation {} {} {}\n".format(x1, height, z1)
                        # angle
                        ang_radians = math.atan2(x2-x1, z2-z1)
                        vehicleString += "  rotation 0 1 0 {}\n".format(ang_radians)
                        vehicleString += "  color 0.18 0.50 0.72\n"
                        vehicleString += "  name \"{}\"\n".format(msg.rob_id)
                        vehicleString += "  controller \"{}\"\n".format("ros_automobile")
                        vehicleString += "  controllerArgs \"--name={} --clock --use-sim-time\"\n".format(msg.rob_id)
                        vehicleString += "  sensorsSlotCenter [\n"
                        vehicleString += "    GPS {\n"
                        vehicleString += "    }\n"
                        vehicleString += "  ]\n"
                        vehicleString += "  }\n"
                        self.rootChildren.importMFNodeFromString(-1, vehicleString)
                        self.robot_ids.append(msg.rob_id)
                        self.count +=1

                if msg.opr == 'rem':
                        if msg.rob_id in self.robot_ids:
                            #self.robot_ids.remove("msg.rob_id")
                            pos = self.robot_ids.index(msg.rob_id)-len(self.robot_ids)
                            self.rootChildren.removeMF(pos)
                            self.robot_ids.remove(msg.rob_id)
