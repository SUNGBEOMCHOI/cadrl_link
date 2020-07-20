# link_waypoint
**waypoint for LINK** inspired by waypoint_nav
https://github.com/nickcharron/waypoint_nav

## Control_base
Since we're using arduino mega board as a motor control board, arduino control code is added (subscribes /cmd_vel).

## OSM for maps
Used osm_cartography package to get maps of the area of interest.

## Local Planner
**teb_local_planner**<br>
Changed base_local_planner to teb_local_planner for better local planning.

## Global Planner
**osm_planner**<br>
Used osm_planner to make global route above the osm lane.

## Simulation
Used **morse** to make 3d simulation environment and added link_model for simulation <br>
Simulation includes link model,outdoor environment and human obstacles.

## Prerequisites
- **e2box_imu_9dofv4** pkg for imu data & magnetormeter data
- **rosserial** pkg for port connection with arduino
- **ultrasonic_ros** pkg for ultrasonic data(A02YYUW)
- **encoder-odometry** pkg for wheel odometry data
- **osm_planner** pkg for global planning in OSM
- **slam_gmapping** for mapping
- **realsense-ros** pkg for realsens D435
- **osm-cartography** pkg to visualize open street map.


## Issues
Still needs some tuning with parameters.
