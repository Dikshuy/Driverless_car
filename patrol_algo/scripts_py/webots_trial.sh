xterm -e "roscore" &
sleep 25
xterm -e "webots /home/dimple/Desktop/Webots_trials/CAIR_webots_v4_working/CAIR_re.wbt" &
sleep 20
xterm -e "python webot_interface_valid_walks.py"
sleep 20
xterm -e "python webots_interface.py" &
sleep 20

