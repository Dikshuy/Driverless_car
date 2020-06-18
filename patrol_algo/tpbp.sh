xterm -e "roscore" &
sleep 5
xterm -e "rosrun mrpp_algos record_data.py '$1'" &
sleep 10
xterm -e "rosrun mrpp_algos tpbp_online.py '$1'" &
sleep 2
xterm -e "rosrun mrpp_algos sumo_sim.py '$1'"
sleep 10
killall xterm & sleep 1
