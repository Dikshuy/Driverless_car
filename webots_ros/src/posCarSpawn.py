from collections import defaultdict
import re
import numpy as np
import math

with open ('/home/dikshant/catkin_ws/src/webots_ros/sumo.net.xml') as f:
    for line in f:
        if "location netOffset=" in line:
            word = line.split(' ')
            for word_l in word:
                if 'netOffset=' in word_l:
                    offset_value = re.findall("\d+\.\d+", word_l)
f.close()
xOffset = float(offset_value[0])
yOffset = float(offset_value[1])
#print (xOffset, yOffset)


xp, yp, zp = [], [], []
edge_dict = defaultdict(dict)
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
            x_co.append(float(coord[0]) + xOffset)
            #Each y added with yOffset
            y_co.append(float(coord[1]) + yOffset)
        else:
            edge_dict[key_value]['x_values'] = x_co
            edge_dict[key_value]['y_values'] = y_co

x_coord = []
y_coord = []
key_val = '142865547_2'
x_coord = edge_dict[key_val]['x_values']
y_coord = edge_dict[key_val]['y_values']
height = 0.4
roll = 0.0
pitch = 0.0

for i in range(np.size(x_coord)):
    pos = [round(-x_coord[i]  + xOffset , 2), height, y_coord[i] - yOffset]
    zp.append(pos[2])
    xp.append(pos[0])

#print (xp[0], xp[1], zp[0], zp[1])

#Finding rotation and translation coordinates
x1 = xp[0]
x2 = xp[1]

y1 = height
y2 = height

z1 = zp[0]
z2 = zp[1]

#print (x1, y1, z1)
#print (x2, y2, z2)

# angle
ang_radians = math.atan2(x2-x1, z2-z1)

string_val = '''\
TeslaModel3 {
  translation %s %s %s
  rotation 0 1 0 %s
  name "agent_0"
  controller "ros_automobile"
  controllerArgs "--name=agent_0 --clock --use-sim-time"
  sensorsSlotCenter [
    GPS {
    }
  ]
}
'''% (x1, y1, z1, ang_radians)
print string_val

