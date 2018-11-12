General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The roadmap_planner package

This package provides two ROS nodes:
1. The [planner node](scripts/prm_planner_node.py) encapsulates the actual [Constraint Programming (CP) based planner](src/roadmap_planner/prm_planner.py)
and exposes part of its functionality via ROS services. To ease the client side usage, we provide the
[PlannerServiceProxies](src/roadmap_planner/service_proxies.py) class with static methods to wrap the service calls.
2. The [execution node](scripts/motion_dispatcher.py) receives solutions from the planner node. It provides the service
**/MOTION_DISPATCHER/BUILD_MOTION_PLAN** to trigger the creation and execution of the calculated plan.
