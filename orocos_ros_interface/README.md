# Orocos_Cogimon_Ros_Interface

This repository provides an interface to communicate  with Coman in the gazebo/Orocos environment by using Ros topics. 

---

#### Requirements:

OS: Ubuntu 14.04
ROS compatibility: Indigo

| Dependencies  |
| ------------- |
| [Orocos-ROS integration](https://github.com/orocos/rtt_ros_integration)         |
| [Coman-environment](http://cogimon.github.io/software/gettingstarted.html)  |


## Set-up:

As the Orocos-Ros integration package is completely independent from the Coman-cogimon enviroment, we need to make sure that both packages are installed correctly and they are fully functional. 
Let's assume that the  Coman-enviroment package is installed here:
```
/vol/cogimon/
```
and the Orocos-Ros integration package is installed here:
```
/home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay
```
and
```
/home/sina/ws/underlay_isolate
```

open the '.bashrc' and add the following lines:

```
source /opt/ros/indigo/setup.bash
export OROCOS_TARGET=gnulinux
source /vol/cogimon/cogimon-minimal-nightly/bin/setup-cogimon-env.sh
export prefix=/vol/cogimon
source /home/sina/ws/underlay_isolated/install_isolated/setup.sh
source /home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/devel/setup.sh
export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:/vol/cogimon/cogimon-minimal-nightly/lib/orocos
```
If  both packages are installed correctly, their examples should run without any problem.

| Examples  |
| ------------- |
| [Orocos-ROS integration](https://github.com/jhu-lcsr/rtt_ros_examples)         |
| [Coman-environment](http://cogimon.github.io/software/gettingstarted.html)  |

## Run:

1- In Terminal 1:

```
roscore
```

2- In Terminal 2:
```
rsb0.14 server
```

3- In Terminal 3
```
deployer-gnulinux -s /home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/Orocos_Cogimon_Ros_Interface/test_orocos.ops
```

4- In Terminal 4

```
gzclient
```

## Test:

In Terminal 5
```
rostopic list 
```

should result in
```
/Coman/Left/Dq/out (The left arm measured velocity) (Published by the interface)
/Coman/Left/T/out  (The left arm measured torque) (Published by the interface)
/Coman/Left/q/out  (The left arm measured position) (Published by the interface)
/Coman/Right/Dq/out(The right arm measured velocity) (Published by the interface)
/Coman/Right/T/out (The right arm measured torque) (Published by the interface)
/Coman/Right/q/out (The right arm measured position) (Published by the interface)
/Coman/Right/in    (The right arm desired position) (Subscribed by the interface)
/Coman/Left/in     (The left arm desired position) (Subscribed by the interface)
/rosout
/rosout_agg
```

To test the simulation in ** position control** mode do the following:
```
rostopic pub /Coman/Right/in std_msgs/Float64MultiArray '{data:[00.5,0,0,0,0,0,0]}'
```
or
```
rostopic pub /Coman/Left/in std_msgs/Float64MultiArray '{data:[00.5,0,0,0,0,0,0]}'
```
