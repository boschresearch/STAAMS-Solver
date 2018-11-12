#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
# TODO: make import more specific
from roadmap_planning_common_msgs.msg import *
from roadmap_planner.service_proxies import PlannerServiceProxies
from roadmap_tools.robot_info import RobotInfo
from roadmap_tools.service_proxies import ServiceProxies
from roadmap_tools.scene_object import SceneObject


def get_pose_and_update_roadmap(name):
    loc_dict_update = {}
    if not name:
        rospy.logwarn("Provided name {} is not valid".format(name))
        return loc_dict_update
    res, suc = ServiceProxies.get_object_poses_by_name_client([name])
    if isinstance(res[0], PoseStamped):
        types, suc2 = ServiceProxies.get_object_type_by_name([name])
        rospy.sleep(1.0)

        res_add = PlannerServiceProxies.add_object_online_client(res[0], name, types[0])
        rospy.sleep(1.0)
    else:
        pass

    loc_list = PlannerServiceProxies.get_locs_for_names([name])
    loc_dict_update[name] = loc_list

    return loc_dict_update


def check_req_nodes_running():
    nodes = rosnode.get_node_names()

    req_nodes = ["/MOTION_DISPATCHER", "/SceneGraphMockupManager", "/prm_planner_wrapper"]

    res = True

    for req_node in req_nodes:
        if req_node not in nodes:
            rospy.logerr("Required node {} is not running. Please start it and retry.".format(req_node))
            res = False

    return res


def main():
    rospy.init_node("OVC_Example_Experiment")

    # retrieve the robot info object for the robot
    robot_info = RobotInfo.getRobotInfo()  # type: RobotInfo

    # check if required nodes are running
    if not check_req_nodes_running():
        rospy.logerr("Exit")
        return 1

    # load the scene and reset the planner
    ServiceProxies.reload_scene_graph_client('0')
    PlannerServiceProxies.reset_planner_client()

    # add location to roadmaps
    # create ovcs
    loc_dict = {}

    # query block objects from SceneGraphMockupManager by type
    blocks = ServiceProxies.get_objects_by_type_client(['block'])[0]  # type: list[SceneObject]
    for block in blocks:
        # query the poses of the objects and add let the planner node add it to the roadmaps
        loc_dict_update = get_pose_and_update_roadmap(block.name)
        loc_dict.update(loc_dict_update)

    # keep dictionary with only parts for OVC creation
    parts = loc_dict.copy()

    # get the robot home poses for each arm and add them to the roadmaps
    name = "home_left"
    loc_dict_update = get_pose_and_update_roadmap(name)
    loc_dict.update(loc_dict_update)

    name = "home_right"
    loc_dict_update = get_pose_and_update_roadmap(name)
    loc_dict.update(loc_dict_update)


    # get the left and right drop location and add them to the roadmaps
    name = "drop_left"
    loc_dict_update = get_pose_and_update_roadmap(name)
    loc_dict.update(loc_dict_update)

    name = "drop_right"
    loc_dict_update = get_pose_and_update_roadmap(name)
    loc_dict.update(loc_dict_update)

    # Setting the start and goal state in the CSP.
    #
    # set the the first configuration variable for the left_arm to home_left
    PlannerServiceProxies.add_state_ct("left_arm", 0, loc_dict["home_left"])
    # set the the last configuration variable for the left_arm to home_left
    PlannerServiceProxies.add_state_ct("left_arm", -1, loc_dict["home_left"])
    PlannerServiceProxies.add_state_ct("right_arm", 0, loc_dict["home_right"])
    PlannerServiceProxies.add_state_ct("right_arm", -1, loc_dict["home_right"])

    # check if planning is feasible
    for name, value in parts.items():
        if len(value) == 0:
            rospy.logerr("location {} can't be reached. Your Problem is infeasible.".format(name))

    # create an OVC per part to put it either to the right or left drop location
    ovc_dict = {}

    for loc_name, loc in parts.items():
        # create a list of lists with names of the locations to consider. Here, one of the robot components should visit
        # first the object location and afterwards either drop_left or drop_right.
        location_names = [[loc_name], ["drop_left", "drop_right"]]
        try:
            # get the integer representation of the previously defined locations.
            loc_domains = [[loc_dict[name][0] if len(loc_dict[name]) > 0 else None for name in names] for names in
                           location_names]  # loc_dict[ovc.location_names]
        except IndexError:
            # if a location is not reachable by any component, the OVC cannot be satisfied and we have to skip it
            rospy.logerr("Location {} is not reachable. Skipping OVC {}".format(name, ovc.name))
            continue

        # create a list of ranges to model the min and max durations at each location
        ranges = [Range() for i in range(len(loc_domains))]
        for r in ranges:
            r.min = 20
            r.max = 1000  # in [.1s]

        # create a list of robot components, which should be considered for the task execution
        groups = robot_info.getGroups()

        # add OVC to solver
        ovc_dict[loc_name] = PlannerServiceProxies.add_ovc(groups=groups, domains=loc_domains, ranges=ranges)
        rospy.loginfo("Added OVC for the group(s) {} to visit {} for {} * 0.1s".format(groups, loc_domains, ranges))

    #
    # start planning
    rospy.loginfo("Starting planner")
    PlannerServiceProxies.solve_client()
    rospy.loginfo("Finished planning and trajecory visualization.")
    return 0


if __name__ == "__main__":
    '''
    This script demonstrates the creation of a simple OVC planning problem. 
    '''
    sys.setrecursionlimit(1000000)

    res = main()

    sys.exit(res)



