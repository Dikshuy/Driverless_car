# mrpp_algos
Note: The TPBP Algorithm is currently configured to the grid scenario. Some modifications are required in the naming conventions to make it more generic.

To use this ros-package you need the following -
- SUMO (to install - sudo apt-get install sumo*)
- Networkx - version 1.11 

To run a new configuration - 'sim_name'
- add an entry 'sim_name' in 'config.txt' with following parameters
    - 'folder' - the folder to store data
    - 'graph' = grid_5_5 (for now)
    - 'num_priority' - Number of priority nodes
    - 'algo' = tpbp_walk (for now)
    - 'algo_params' - The coefficients of the cost function
    - 'min_time' - Total simulation time

- create database of valid walks by running
    - python scripts_py/valid_walks.py sim_name

- For a simulation run execute
    - ./tpbp.sh sim_name (Click play in the SUMO gui)

The outputs generated are as follows
- sim_data.in
    - Data is recorded in 3 successive lines (Time Stamp, Node Ids, Robot Ids)
- node_idleness.csv
    - Contains idleness of each node at every time instance
-robot_*.in
    - Each row represents a Time and Node pair representing the corresponding robot's walk
