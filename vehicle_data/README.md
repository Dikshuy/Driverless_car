##### Trajectory Tracking Implementation on CAIR Map Segment.


Inside "scripts" folder contains "CAIR_seg.py" which deals with the Trajectory Tracking Implementation on Road segment of CAIR Map. "graph1.py" which deals with the plotting of reference path and actual path for car.

Inside "src" folder contains "webots_auto_wrapper.py" which act as interface for ROS node which controls the object in webots and webots simulator.

Run "CAIR_mod.wbt". 

Give Proper Path of "CAIR_seg.py", "sumo.net.xml" and "edge_info.in" to generate array of way points.



For Running the nodes and world file use this commands in terminals,

Terminal 1: $ roscore

Terminal 2: $ rosparam set use_sim_time true

Terminal 3: $ rosrun vehicle_data webots_auto_wrapper.py

Terminal 4: $ rosrun vehicle_data graph1.py

Terminal 5: $ rosrun vehicle_data CAIR_seg.py

Terminal 6: $ webots /home/dimpleb/catkin_ws/src/vehicle_data/worlds/CAIR_mod.wbt



