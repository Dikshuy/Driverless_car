rm -f a.txt

xterm -e "roscore" &
sleep 5
xterm -e "rosparam set use_sim_time true" &
sleep 5
xterm -e "rosrun vehicle_data webots_auto_wrapper.py" &
sleep 1
xterm -e "rosrun vehicle_data CAIR_seg.py" &
sleep 1
xterm -e "webots /home/dikshant/catkin_ws/src/webots_ros/worlds/CAIR_mod.wbt"
sleep 15

