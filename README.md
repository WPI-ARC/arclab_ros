arclab_ros
==========
General ROS Packages and other robot-agnostic code developed at ARC lab.

This repository contains Catkinized packages for ROS Groovy+. It will not build on ROS Fuerte or earlier!


Repository structure
--------------------


Stability and development status
--------------------------------

`simple_robot_kinematics` - Package has been catkinized, awaiting testing with MoveIt! on the PR2 in simulation and real hardware.


Build and usage instructions
------------------------------
To build all packages in this repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make
```
To build a particular package in the repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make --pkg <package name>
```
To use, you must source the workspace:

```
(in the surrounding Catkin workspace directory)
$ source devel/setup.bash
```

For usage information and instructions on running components of these packages together, see the repository [Wiki](https://github.com/WPI-ARC/arclab_ros/wiki).
