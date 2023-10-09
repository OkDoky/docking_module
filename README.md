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

# parameter name
path_displacement -> straight section length

robot_size

marker_displacement

---
robot_size + marker_displacement = distance from marker
---

# 도커 실행방법
---
nvidia-docker run -it -d --restart unless-stopped --gpus all -v /run/user/1000:/run/user/1000 -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix:ro --privileged --shm-size=256m -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e XAUTORITY=/tmp/.docker.xauth -e XDG_RUNTIME_DIR=/run/user/1000 --name "docking" docking:latest /bin/bash
---
