#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import rospy
import cPickle as pickle

import moveit_commander

from moveit_msgs.msg import RobotState, RobotTrajectory
import moveit_msgs.srv

from roadmap_planning_common_msgs.srv import BuildMotionAddTrajectory, BuildMotionAddTrajectoryResponse, BuildMotionAddTrajectoryRequest, \
    StringQuery, StringQueryResponse, StringQueryRequest
from std_srvs.srv import EmptyResponse, Empty, EmptyRequest
from trajectory_msgs.msg import JointTrajectoryPoint


from roadmap_tools.prm import RoadMap
import numpy as np

import roadmap_tools.file_handler as file_handler

import bisect

from roadmap_tools.prm_factory import RoadMapFactory

from roadmap_tools.robot_info import RobotInfo


class motion_dispatcher:

    def __init__(self):
        rospy.init_node("MOTION_DISPATCHER")

        parameter_ns = "SolverSetup"

        self.exp_host = parameter_ns

        self.robot_info = RobotInfo.getRobotInfo()

        self.file_handler = file_handler.file_handler()

        # send_trajectory_srv = rospy.ServiceProxy('MOTION_DISPATCHER/ADD_TRAJECTORY', BuildMotionAddTrajectory)

        self.motion_requests = {}  # type: dict[str, BuildMotionAddTrajectoryRequest]
        self.roadmaps = {}  # type: dict[str, RoadMap]
        self.MoveGroupComanders = {}  # type: dict[str, moveit_commander.MoveGroupCommander]

        self.motion_plan_times = []
        self.motion_plan_pos = {}  # type: dict[str, list[int]]

        self.motion_plan_valid = False

        self.motion_plan = RobotTrajectory()

        self.SRV_ADD_TRAJECTORY = rospy.Service("MOTION_DISPATCHER" + '/ADD_TRAJECTORY', BuildMotionAddTrajectory, self.handle_add_trajectory)

        self.SRV_BUILD_MOTION_PLAN = rospy.Service("MOTION_DISPATCHER" + '/BUILD_MOTION_PLAN', Empty, self.handle_build_plan)

        self.SRV_GO_TO_NODE = rospy.Service("MOTION_DISPATCHER" + '/MOVE_TO_NODE', StringQuery,
                                                   self.handle_go_tp_node)

        rospy.spin()

    def load_roadmap(self, file_name, group_name="right_arm"):
        if group_name in self.roadmaps.keys():
            del self.roadmaps[group_name]
            rospy.loginfo("Deleted RoadMap for {} to load another one for same group.".format(group_name))
        # self.roadmaps[group_name] = RoadMap(self.MoveGroupComanders[group_name], file_name, group_name)
        self.roadmaps[group_name] = RoadMapFactory.load_prm_from_database(file_name)
        rospy.loginfo("Loaded RoadMap for {}.".format(group_name))

    def handle_add_trajectory(self, req):
        # type: (BuildMotionAddTrajectoryRequest)->BuildMotionAddTrajectoryResponse
        res = BuildMotionAddTrajectoryResponse()

        if req.prm_pickle == "":
            pass
        else:
            rm = pickle.loads(req.prm_pickle)  # type: RoadMap
            assert isinstance(rm, RoadMap)
            self.roadmaps[req.move_group] = rm
            rospy.loginfo("Transferred Roadmap via message: {}".format(rm.get_fingerprint()))

        if req.move_group in self.motion_requests.keys():
            del self.motion_requests[req.move_group]
            rospy.logwarn("group already exists: delete motion request for {}".format(req.move_group))
            res.msg += "group already exists: overwrite motion request \n"
        self.motion_requests[req.move_group] = req
        self.motion_plan_valid = False


        rospy.loginfo("Adding motion request for group {}".format(req.move_group))
        res.success = True
        res.msg += "received motion requests: \n"
        for name in self.motion_requests.keys():
            res.msg += "\t{}\n".format(name)

        return res

    def display_plan(self, group):
        plan = self.motion_requests[group]  # type: BuildMotionAddTrajectoryRequest

        for pos in plan.prm_pos:
            pose = self.roadmaps[group].get_eef_pose(pos)
            # TODO: work on a visualization of plans

    def check_plan(self, plan, sv=None, robot_com=None):
        # type: (RobotTrajectory, kinematics_interface.StateValidity) -> bool
        # plan.joint_trajectory.points
        # sv = kinematics_interface.StateValidity()
        sv = self.robot_info.getSV()
        display_state_publisher = rospy.Publisher(
            '/move_group/display_robot_state',
            moveit_msgs.msg.DisplayRobotState,
            queue_size=20)
        collision_states = []
        for state in plan.joint_trajectory.points:
            rs = RobotState()
            rs.joint_state.name = plan.joint_trajectory.joint_names
            rs.joint_state.position = state.positions
            res = sv.getStateValidity(rs, self.robot_info.getCompleteGroup())

            state_msg = moveit_msgs.msg.DisplayRobotState()
            rstate = robot_com.get_current_state()
            state_msg.state.joint_state = rstate.joint_state
            state_msg.state.joint_state.position = state.positions
            display_state_publisher.publish(state_msg)

            if not res.valid:
                collision_states.append(rs)

        print collision_states

        if len(collision_states) > 0:
            return False
        else:
            return True

    def handle_build_plan(self, req):
        # type: (EmptyRequest)->EmptyResponse
        res = EmptyResponse()
        # build combined motion plan and interpolate missing points
        self.ensure_build_plan_requirements()
        self.motion_plan = self.build_plan()

        rospy.sleep(2.0)

        self.move_group_to_roadmap_node(self.motion_requests.keys()[0],
                                        self.motion_plan_pos[self.motion_requests.keys()[0]][0])
        rospy.sleep(2.0)

        self.move_group_to_roadmap_node(self.motion_requests.keys()[-1],
                                        self.motion_plan_pos[self.motion_requests.keys()[-1]][0])

        wb = moveit_commander.MoveGroupCommander(self.robot_info.getCompleteGroup())

        rospy.sleep(2.0)

        wb.execute(self.motion_plan)

        return res

    def handle_go_tp_node(self, req):
        # type: (StringQueryRequest) -> StringQueryResponse
        res = StringQueryResponse()


        group, roadmap_name, node = req.input
        self.load_roadmap(roadmap_name, group)
        node = int(node)
        self.ensure_build_plan_requirements()

        self.move_group_to_roadmap_node(group, node)

        res.output.append("Moved {} to node {} in {}".format(group, node, self.roadmaps[group]))
        res.success = True

        return res

    def move_group_to_roadmap_node(self, group_name="right_arm", vertex=0):
        joint_names_gr = self.MoveGroupComanders[group_name].get_active_joints()
        joint_names_prm, positions = self.getJointAnglesFromPrmNode(vertex, group_name)

        jv_goal = [0] * len(joint_names_gr)

        for name in joint_names_gr:
            i_prm = joint_names_prm.index(name)
            i_gr = joint_names_gr.index(name)
            jv_goal[i_gr] = positions[i_prm]

        self.MoveGroupComanders[group_name].clear_pose_targets()
        self.MoveGroupComanders[group_name].set_start_state_to_current_state()
        self.MoveGroupComanders[group_name].set_joint_value_target(jv_goal)
        plan = self.MoveGroupComanders[group_name].plan()
        print self.MoveGroupComanders[group_name].get_goal_joint_tolerance()
        ret = self.MoveGroupComanders[group_name].execute(plan, True)

        if not ret:
            rs = RobotState()
            rs.joint_state.name = joint_names_prm
            rs.joint_state.position = positions
            sv = self.robot_info.getSV()
            # sv = kinematics_interface.StateValidity()
            stateVal = sv.getStateValidity(rs, group_name=group_name)
            print stateVal

        print self.MoveGroupComanders[group_name].get_current_joint_values()
        print jv_goal




    def ensure_build_plan_requirements(self):
        for group in self.motion_requests.keys():
            if group not in self.MoveGroupComanders.keys():
                self.MoveGroupComanders[group] = moveit_commander.MoveGroupCommander(group)
            if group not in self.roadmaps:
                self.load_roadmap(self.motion_requests[group].prm_name, group)


    def build_plan(self):
        # TODO: bring parameters outside
        SPEED_FACTOR = 0.1
        SECS2NS = 1000000000  # 1 Mrd ns/sec

        self.motion_plan_times = []
        self.motion_plan_pos = {}  # type: dict[str, list[int]]

        self.motion_plan_valid = False

        self.motion_plan = RobotTrajectory()

        extension_index_tuples = {}  # type: dict[str, list[Tuple]]
        for group in self.motion_requests.keys():
            extension_index_tuples[group] = []
            for index, pos in enumerate(self.motion_requests[group].prm_pos):
                i2 = index + 1
                if i2 >= len(self.motion_requests[group].prm_pos):
                    break
                else:
                    pos2 = self.motion_requests[group].prm_pos[i2]
                path = self.roadmaps[group].find_path_prm(pos, pos2, True)
                if len(path) > 2:
                    # positions are not adjacent
                    extension_index_tuples[group].append((index, i2, path))

        for group in self.motion_requests.keys():
            extension_index_tuples[group].reverse()
            for i1, i2, path in extension_index_tuples[group]:
                time_i1 = self.motion_requests[group].time[i1]
                time_i2 = self.motion_requests[group].time[i2]
                times_list = len(path) * [0]
                times_list[0] = time_i1
                for i in range(1, len(path)):
                    times_list[i] = times_list[0] + self.roadmaps[group].shortest_distance(source=path[0],
                                                                                           target=path[i],
                                                                                           weights=True) / SPEED_FACTOR
                times_list[-1] = time_i2
                first_pos = self.motion_requests[group].prm_pos[:i1]
                second_pos = self.motion_requests[group].prm_pos[i2 + 1:]
                first_time = self.motion_requests[group].time[:i1]
                second_time = self.motion_requests[group].time[i2 + 1:]
                self.motion_requests[group].prm_pos = first_pos + tuple(path) + second_pos
                self.motion_requests[group].time = first_time + tuple(times_list) + second_time

                # TODO: finish and verify the expansion of implicit plans
                # print path
                # print times_list
                # print self.motion_requests[group].prm_pos
                # print self.motion_requests[group].time



        self.motion_plan_times = []
        for group in self.motion_requests.keys():
            for time_step in self.motion_requests[group].time:
                # print(time_step)
                if type(time_step) == float:
                    print("why float?")
                if time_step not in self.motion_plan_times:
                    self.motion_plan_times.append(time_step)

        self.motion_plan_times.sort()

        for group in self.motion_requests.keys():
            self.motion_plan_pos[group] = []
        for time_step in self.motion_plan_times:
            for group in self.motion_requests.keys():
                if time_step in self.motion_requests[group].time:
                    index = self.motion_requests[group].time.index(time_step)
                    pos = self.motion_requests[group].prm_pos[index]
                    self.motion_plan_pos[group].append(pos)
                else:  # interpolate
                    times_available = self.motion_requests[group].time
                    print "We need interpolation"
                    i = bisect.bisect_left(times_available, time_step)
                    if i:
                        index = i-1  # times_available[i-1]
                        if index + 1 >= self.motion_requests[group].prm_pos.__len__():
                            print "Index out of Bound?"
                        print "Index: {}".format(index)
                        print "i: {}".format(i)
                        print "i-1: {}".format(i - 1)
                        print "len(prm_pos): {}".format(self.motion_requests[group].prm_pos.__len__())
                        pos = self.motion_requests[group].prm_pos[i-1]

                        pos2 = self.motion_requests[group].prm_pos[i]
                        if pos == pos2:
                            self.motion_plan_pos[group].append(pos)
                        else:
                            part = (1.0 * time_step - times_available[i - 1]) / (
                                    times_available[i] - times_available[i - 1])
                            self.motion_plan_pos[group].append([pos, pos2, part])
                        rospy.loginfo("take the pos for the next smaller time step - they are eqal. No need to interpolate")
                    else:
                        rospy.logerr("No Value assigned to plan: panic!")
                    # take the pos for the next smaller value
                    # return False

        print self.motion_plan_pos
        print self.motion_plan_times
        self.motion_plan_valid = True

        # ra = moveit_commander.MoveGroupCommander("right_arm")
        # la = moveit_commander.MoveGroupCommander("left_arm")
        # ra.set_pose_target(ra.get_random_pose())
        # plan = ra.plan()
        # pl = moveit_msgs.srv.GetMotionPlanRequest()
        # ts = rospy.Duration(plan.joint_trajectory.points[1].time_from_start.nsecs)
        # print ts.to_sec()
        plan = RobotTrajectory()  # type: RobotTrajectory
        # plan.joint_trajectory.header.frame_id = "WAIST"
        plan.joint_trajectory.header.frame_id = self.MoveGroupComanders.values()[-1].get_planning_frame()
        # adding all joint names of the groups to the plan
        for com in self.MoveGroupComanders.values():
            plan.joint_trajectory.joint_names.extend(com.get_active_joints())

        # build a JointTrajectoryPoint for each time_step
        for g in self.motion_requests.keys():
            print "Nr. Poses for {}: {}".format(g, len(self.motion_plan_pos[g]))
            print "Nr. of time steps: {}".format(len(self.motion_plan_times))

            assert len(self.motion_plan_pos[g]) == len(self.motion_plan_times)

        for time_index, time_step in enumerate(self.motion_plan_times):
            # time_index = self.motion_plan_times.index(time_step)
            pt = JointTrajectoryPoint()
            # pt.time_from_start = rospy.Duration(0, long(SECS2NS * SPEED_FACTOR * time_step *10))  # .set(time_step, 0)
            # pt.time_from_start = rospy.Duration(0, int(SECS2NS * SPEED_FACTOR * time_step *10))  # .set(time_step, 0)
            pt.time_from_start = rospy.Duration(0, int(SECS2NS * SPEED_FACTOR * time_step))
            # pt.time_from_start.nsecs = long(SECS2NS * SPEED_FACTOR * time_step)  # .set(time_step, 0)

            # getting the joint positions from the Roadmaps
            pt.positions = [0] * len(plan.joint_trajectory.joint_names)
            for _group in self.motion_requests.keys():
                if type(self.motion_plan_pos[_group][time_index]) is list:
                    interpolation_triple = self.motion_plan_pos[_group][time_index]
                    node0 = interpolation_triple[0]
                    node1 = interpolation_triple[1]
                    part = interpolation_triple[2]
                    names, positions0 = self.getJointAnglesFromPrmNode(node0, _group)
                    names, positions1 = self.getJointAnglesFromPrmNode(node1, _group)
                    positions = np.array(positions0) + part * (np.array(positions1)-np.array(positions0))
                    positions = positions.tolist()
                    # print positions
                else:
                    node = self.motion_plan_pos[_group][time_index]
                    names, positions = self.getJointAnglesFromPrmNode(node, _group)
                # node = self.motion_requests[_group].prm_pos[time_index]
                # names, positions = self.getJointAnglesFromPrmNode(node, _group)
                # joints = self.MoveGroupComanders[_group].get_active_joints()
                for joint_name in self.MoveGroupComanders[_group].get_active_joints():
                    i_prm = names.index(joint_name)
                    i_plan = plan.joint_trajectory.joint_names.index(joint_name)
                    pt.positions[i_plan] = positions[i_prm]
            plan.joint_trajectory.points.append(pt)

        return plan

    def getJointAnglesFromPrmNode(self, node, group_name="right_arm"):
        # type: (int) -> (list[str], list[double])
        rs = self.roadmaps[group_name].get_robot_state_for_vertex(node)  # type: RobotState
        return rs.joint_state.name, rs.joint_state.position


if __name__ == '__main__':
    print "motion dispatcher start"
    mdp = motion_dispatcher()
