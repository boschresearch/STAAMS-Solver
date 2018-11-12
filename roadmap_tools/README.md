General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

## Saving joint states as seeds

The system depends on seeds for the kinematic solvers, which can be created by moving the robot into a state and calling:
```bash
rosrun roadmap_tools save_joint_states.py "robot_name" "off/ik_seed"
```
Replace robot_name with the robot_name as it is specified on the parameter server. Call this once with off, when the robot
is in a state in which each group does not restrict the movements of any other group. Catt it again with ik_seed with a state,
which is close to good configurations for the expected manipulations.
