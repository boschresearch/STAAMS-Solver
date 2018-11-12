General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The nextage_cp_adapter package

To use the STAAMS-Solver for your robot, you need to provide certain information about the robot. The nextage_cp_adapter package provides the configuration for the [Nextage Open robot by KaWaDa Industries](http://nextage.kawada.jp/en/). 

The following elements are present:

- the [package.xml](package.xml) defines the software dependencies for robot dependent ROS packages, such that they can
be installed using [rosdep](http://wiki.ros.org/rosdep) or built in the same or shadowed catkin workspace.
- the [kawada_robot_info.py](src/nextage_cp_adapter/kawada_robot_info.py) inherits from RobotInfo. You have to implement
all unimplemented methods of the RobotInfo class. The reason for not using the ROS parameter server only is, that we
need to distribute python objects and not only primitive data types.
- the [kinematics_interface](src/nextage_cp_adapter/kinematics_interface.py) provides kinematic functionality and
collision checking functionality, which is accessible through the KawadaRobotInfo class.
- set the ROS parameter SolverSetup/ROBOT_INFO_CLASS to YOUR_PACKAGE_NAME.YOUR_ROBOT_INFO_CLASS_MODULE in a script like
[this](scripts/write_solver_params_kawada_base_rm.py) with
```python
rospy.set_param("SolverSetup/ROBOT_INFO_CLASS", "nextage_cp_adapter.kawada_robot_info")
```
- write a startup routine like [startup.py](startup.py), which brings up your robot or robot simulation and also loads
the parameters to the parameter server. The [launch files](launch) launch [MoveIt!](https://moveit.ros.org/) and
[RVIZ](http://wiki.ros.org/rviz) with a [custom RVIZ config](cfg/nextage_cp.rviz).
