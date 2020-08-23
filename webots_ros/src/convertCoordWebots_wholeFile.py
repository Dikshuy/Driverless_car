import re
from collections import defaultdict

#Read offset values from net file.
with open ('./CAIR_mod_net/sumo.net.xml') as f:
    for line in f:
        if "location netOffset=" in line:
            word = line.split(' ')
            for word_l in word:
                if 'netOffset=' in word_l:
                    offset_value = re.findall("\d+\.\d+", word_l)
f.close()
xOffset = float(offset_value[0])
yOffset = float(offset_value[1])

xp, yp = [], []
edge_dict = defaultdict(dict)
#Read values from a file corresponding to the edge
with open ('./CAIR_mod_net/CAIR_mod_edge_info.in') as f1:
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

file1 = open("myfile.txt", "a")
for key, value in edge_dict.items() :
    x_coord = []
    y_coord = []
    str_val = ""
    key_val = key
    x_coord = edge_dict[key_val]['x_values']
    y_coord = edge_dict[key_val]['y_values']
    color_val_pos = "          0.8 0.0 0.0"
    color_val_neg = "          0.8 0.8 0.2"
    if key[0] == '-':
        color_val = color_val_neg
    else:
        color_val = color_val_pos


    #wps
    height = 0.0
    '''
    for i in range (len(x_coord)):
        print (-x_coord[i]+xOffset, ' ', height, ' ', y_coord[i]-yOffset)

    print ("\n\n\n")
    for i in range (len(x_coord)):
        print ("0.8 0.0 0.0")
    '''

    str_val_1 = '''\
    DEF POINT_SET_%s Transform {
      translation 0.0 0.0 0.0
      children [
        Shape {
          geometry PointSet {
            color Color {
              color [
    '''% (key_val)

    #print (str_val_1)

    str_val_2 = ""
    for i in range (len(x_coord)):
    	if i == len(x_coord)-1:
    		str_val_2 += color_val
    	else:
    		str_val_2 += color_val+'\n'

    #print (str_val_2)

    str_val_3 = '''
              ]
            }
            coord Coordinate {
              point [
    '''

    str_val_4 = ""
    for i in range (len(x_coord)):
    	if i == len(x_coord)-1:
    		str_val_4 += "          "+str(-x_coord[i]+xOffset) + ' ' + str(height) + ' '+str(y_coord[i]-yOffset)
    	else:
    		str_val_4 += "          "+str(-x_coord[i]+xOffset) + ' ' + str(height) + ' '+str(y_coord[i]-yOffset)+'\n'
    str_val_5 = '''
              ]
            }
          }
        }
      ]
    }
    '''
    str_val = str_val_1+str_val_2+str_val_3+str_val_4+str_val_5+'\n'
    file1.write(str_val)
file1.close()
