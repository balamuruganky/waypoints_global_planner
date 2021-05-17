# Waypoints Global Planner

## Abstract

Suppose the waypoints given for the predefined path in the form of YAML file or nav_msgs::Path message to track the path using any path tracking local planner, the global planner shall publish either the YAML or the nav_msgs::Path to make use of the Move Base plugin.

Waypoints global planner designed to publish the path from
1) YAML file
2) nav_msgs::Path buffer

## Path Planning

Python path planner shall create waypoints based on user input points. Please refer "scripts" folder for more details. 

## How to compile

It is assumed that catkin workspace is available already to compile.
  - cd catkin_ws/src
  - git clone https://github.com/balamuruganky/waypoints_global_planner
  - cd ..
  - catkin_make

## Config
  - planner_frequency: 0.0
  - planner_patience: 1.0
  - base_global_planner: "waypoints_global_planner/WaypointsGlobalPlanner"
  - WaypointsGlobalPlanner:
      - map_frame: "map"

## How to Launch

Set the base_global_planner parameter in Move Base navigation stack and publish the path using either of the listed launch files
  - roslaunch waypoints_global_planner desired_path_provider.launch
  - roslaunch waypoints_global_planner yaml_file_path_provider.launch

## Caveat

It is assumed that there are no obstracles present between start and the goal position and it is a static plan for path tracking robots.
