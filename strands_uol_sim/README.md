strands_uol_sim
===============

This package contains files that are necessary for running STRANDS simulations on the University of Lincoln environments.


### Setting up Autonomous Patrolling Simulation

1. Calibrate charging station parameters:
   * Launch strands_datacentre:
           ```
           roslaunch strands_datacentre datacentre.launch
           
           ```
   * Launch UOL_MHT simulation:
           ```
           roslaunch strands_morse uol_mht_morse.launch
           
           ```
   * Launch scitos_docking:
           ```
           roslaunch scitos_docking charging.launch
           
           ```
   * Drive the robot to the charging station
   * Calibrate charging parameters running:
           ```
           rosrun scitos_docking visual_charging_client calibrate 100
           
           ```
2. Insert waypoints on database:
   * Launch strands_datacentre:
           ```
           roslaunch strands_datacentre datacentre.launch
           
           ```
   * Insert waypoints in DB
           ```
           rosrun waypoint_recorder insert_in_db.py /opt/strands/strands_catkin_ws/src/strands_morse/uol/maps/uol_mht_sim_III.csv
           
           ```
   NOTE: You can also create your own waypoints folowing the steps descripted in: https://github.com/strands-project/autonomous_patrolling

### Launching Autonomous Patrolling Simulation

If all previous steps are done launch simulation by running:

       ```
       roslaunch strands_uol_sim mht_sim.launch
   
       ```
