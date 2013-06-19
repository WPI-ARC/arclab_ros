arclab_ros
==========
General ROS Packages and other robot-agnostic code developed at ARC lab.

This repository contains Catkinized packages for ROS Groovy+. It will not build on ROS Fuerte or earlier!


Repository structure
--------------------

`simple_robot_kinematics` - Provides a simple Python interface to the forward and inverse kinematics services of a robot. Formerly supported the PR2 using `arm_navigation`, now supports the PR2 using MoveIt!. Support will be added for the Baxter robot in the near future.

`generic_tf_broadcaster` - Provides a simple fixed-frame TF broadcaster configurable via launch file. This is used extensively in other lab projects.

`ros_email` - Provides a SMTP email client for ROS that supports both simple text-only emails and emails with attached files. This package allows your robot to send email notifications and tweet both text and pictures using widely available email->Twitter services.

`UDEV_rules` - Provides UDEV rules to set device access permissions for devices used in lab, and a script to install them.


Stability and development status
--------------------------------

`simple_robot_kinematics` - Package has been catkinized, awaiting testing with MoveIt! on the PR2 in simulation and real hardware.

`generic_tf_broadcaster` - Package is stable and tested.

`UDEV_rules` - All current rules are stable and tested.

`ros_email` - Package is stable and tested.


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
