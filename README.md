# STAAMS-Solver repository

This repository provides a Simultaneous Task Allocation And Motion Scheduling (STAAMS) Solver based on Constraint Programming (CP) including a running example. The provided method is described in paper submitted to the Robotics and Automation Letters by Behrens et al [1].

[1] Jan K. Behrens, Ralph Lange, Masoumeh Mansouri. A Constraint Programming Approach to Simultaneous Task Allocation and Motion Scheduling for Industrial Dual-Arm Manipulation Tasks. Submitted to RA-L 2019


## Requirements, how to build, test, install, use, etc.

We developed and tested using Ubuntu 16.04 and ROS Kinetic. Core dependecies are:

- [Google OR-Tools](https://developers.google.com/optimization/)
- [Graph-tool](https://graph-tool.skewed.de/)
- [lmdb](http://www.lmdb.tech/doc/)

```bash
echo "deb http://downloads.skewed.de/apt/xenial xenial universe" >> /etc/apt/sources.list
echo "deb-src http://downloads.skewed.de/apt/xenial xenial universe" >> /etc/apt/sources.list

apt-key adv --keyserver pgp.skewed.de --recv-key 612DEFB798507F25

apt-get update && apt-get install -y --allow-unauthenticated \
    build-essential python-pip python-graph-tool liblmdb

pip install --upgrade lmdb pip catkin_tools ortools plotly --user

```
Clone the repository into a ROS workspace. Execute from inside the src folder
```bash
git clone https://github.com/boschresearch/STAAMS-Solver.git
rosdep install --from-paths . --ignore-src -r -y
```

We use the install space for some example data. To enable the use of the install space, execute:
```bash
catkin config --install
```
Build it using [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/).
```bash
catkin build
```

## Running the examples

### Four blocks example
This will start a gazebo simulation, RVIZ visualization, the planning infrastructure (scenegraph, planner, and motion executor), and a [script](cp_planning_examples/scripts/four_blocks_01.py) using these to plan a small example of moving four blocks from the table.

```bash
rosrun nextage_cp_adapter startup.py
rosrun cp_planning_examples example_scene_manager.py
rosrun roadmap_planner prm_planner_node.py
rosrun roadmap_planner motion_dispatcher.py
rosrun cp_planning_examples four_blocks_01.py
```

## Package structure

- [3rd-party](3rd-party/) contains dependencies which are not available as Ubuntu package
- [cp_planning_examples](cp_planning_examples/) provides examples for the solver usage
- [nextage_cp_adapter](nextage_cp_adapter/README.md/) contains a ros package defining the robot dependencies and adapting it for the STAAMS solver
- [roadmap_planner](roadmap_planner/) is a ROS package which defines the planner, a wrapping node, the motion dispatcher (moves the robot in simulation)
- [roadmap_planner_tools](roadmap_planner_tools/) is a ROS package which holds misc tools for the roadmap planner. Mainly a helper script to build a planner input data structure is provided
- [roadmap_planner_common_msgs](roadmap_planning_common_msgs/) defines messages and services
- [roadmap_tools](roadmap_tools/) holds data structures and ros nodes (scene-graph, roadmap, visualization)

## License
STAAMS-Solver is open-sourced under the BSD-3-Clause license. See the [LICENSE](LICENSE) file for details.

For a list of other open source components included in Benchmarks, see the file 3rd-party-licenses.txt.