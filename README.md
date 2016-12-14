# bimanual-catching-coman
human-inspired bimanual catching with Coman

## Requirements

| Dependencies  |
| ------------- |
| [Cogimon environment](http://cogimon.github.io/software/gettingstarted.html)  |
| [OROCOS-ROS integration](https://github.com/orocos/rtt_ros_integration)     |
| [OROCOS-Cogimon-ROS interface](https://github.com/epfl-lasa/Orocos_Cogimon_Ros_Interface)  |
| [Robot-toolkit](https://github.com/epfl-lasa/robot-toolkit)  |




## How to run

Terminal 1:

```
roscore
```

Terminal 2:
```
rosrun ball_motion_estimation ball_motion_estimation_node
```

Terminal 3:
```
deployer-gnulinux -s /home/kevin/catkin_ws/src/bimanual_catching/orocos_ros_interface/orocos_ros_interface.ops
```

Terminal 4:

```
gzclient
```

Terminal 5:

```
(robot-toolkit directory) ./bin/robot_simulator --config packages/bimanual_catching_package/bimanual_catching_module
```
