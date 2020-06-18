#This is for doing robot add/remove/telepoerate/...  operations
Header header
uint8 operation
string robot_id
string start_node_id
bool rem_check

uint8 ADD_ROBOT_TO_SYSTEM=1
uint8 REMOVE_ROBOT_FROM_SYSTEM=2
uint8 ADD_TO_AUTONOMOUS_PATROL=3
uint8 REMOVE_FROM_AUTONOMOUS_PATROL=4
uint8 TELEOPERATE=5 #if robot is in patrol, it will be removed from patrol and set to teleoperate.
uint8 ADD_ROBOT_TO_CHARGING=6
uint8 SET_ROBOT_TO_IDLE=7
uint8 SENDING_ROBOT_FOR_CHARGING=8
uint8 REMOVE_FROM_AUTONOMOUS_PATROL_AFTER_COMPLETION = 9
