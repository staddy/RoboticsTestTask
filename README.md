Install ROS Noetic: https://wiki.ros.org/ROS/Installation

In the project root:

1st terminal:
```
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roscore
```

2nd terminal:
```
source devel/setup.bash
rosrun robots_interface_server position_control_node
```

3rd terminal:
```
source devel/setup.bash
rosrun robot_client robot_client_node
```
