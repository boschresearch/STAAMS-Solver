General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# cp_planning_examples

This package holds a runnable example using the STAAMS planner in a ROS environment. 

- [Four Blocks](scripts/four_blocks_01.py) is a simple example how tasks are created and planned.
- [Example Scene Object Factory](src/cp_planning_examples/example_scene_object_factory.py) defines the object
types for the scene. Also, the example scene is defined in the script. Note, that the scene is defined relative
to a frame "workpiece" which has to placed in the world later.
- [Example Scene Manager](scripts/example_scene_manager.py) defines a complete scene using a scene object factory and
transforms for the placement of the robot and the workpiece.
