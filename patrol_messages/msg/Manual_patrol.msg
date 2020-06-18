uint32 seq
string robot_id
string[] patrol_route #space separated sequence of waypoint-ids as per rndf
bool remove_route_from_autonomous_patrol #

uint8 manual_operation #this value define whether to add or remove based on following two literals
uint8 ADD_TO_MANUAL_PATROL = 1
uint8 REMOVE_FROM_MANUAL_PATROL = 2
