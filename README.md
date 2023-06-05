# docking_module
robotics team project. mobile robot docking module using marker
# command
```
roslaunch docking_gazebo robot.launch
roslaunch marker_detecting detecting_marker.launch marker_id:=7
roslaunch docking_planner generate_path.launch 
roslaunch docking_tracker path_tracking.launch
```


# topic name
compute_Path(msg type - Path)
