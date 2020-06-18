Header header
string folder #what is it? Define. Remvo it after 
string graph #rename it

string path_to_rndf #absolute path to rndf file
int8 num_priority #REMOVE IT
string priority_nodes #space separated string id of nodes
string time_periods #space separated time period in minutes

int8 num_robots #total number of robots. REMOVE IT

string robot_ids #space separated list of robot_id strings. 

string algo
string algo_params #REMOVE. not required
int64 min_time #for simulation purpose only

string intrusion_response_policy
string intrusion_response_policy_param #space separated list of intrusion response behaivour parameters, if any

patrol_messages/Manual_patrol[] manual_patrols #Assigned list of manual patrols

string ALGORITHM_TPBP=tpbp_walk
string ALGORITHM_CBLS=CBLS

