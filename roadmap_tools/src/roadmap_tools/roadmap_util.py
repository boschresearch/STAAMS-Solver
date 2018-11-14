#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import itertools
import time
from graph_tool import Vertex

from moveit_msgs.msg import RobotTrajectory
from roadmap_tools.prm import RoadMap
import math
import moveit_commander
import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint


class RoadmapUtils:

    def __init__(self, sv, fk, ik):

        self.sv = sv  # type: StateValidity
        self.fk = fk  # type: ForwardKinematics
        self.ik = ik  # type: InverseKinematics

        self.mgroups = {}  # type: dict[str, moveit_commander.MoveGroupCommander]

    def get_mgroup(self, name):
        try:
            return self.mgroups[name]
        except KeyError:
            self.mgroups[name] = moveit_commander.MoveGroupCommander(name)
            return self.mgroups[name]

    def get_robot_state(self, eef_pose, link, group):
        resp = self.ik.getIK(group, link, eef_pose)

    # returns a list of vertices with ascending distances to the given RobotState
    def get_neighborhood(self, prm, q1, dist_max=1.0):
        # type: (RoadMap, Vertex) -> (list[Vertex], list[float])
        assert isinstance(q1, Vertex)
        assert isinstance(prm, RoadMap)

        res = []
        fin_res = []
        for s in prm.vertices():
            if prm.get_robot_state_for_vertex(s) is prm.get_robot_state_for_vertex(q1):
                continue
            # dist = self.distance(self.prm.get_robot_state_for_vertex(s), self.prm.get_robot_state_for_vertex(q1),
            #                      "time")
            dist = self.point_point_dist(prm, q1, s)
            # dist = self.distance(self.prm.get_robot_state_for_vertex(s), self.prm.get_robot_state_for_vertex(q1),
            #                      "ed")
            # dist = self.distance(self.prm.get_robot_state_for_vertex(s), self.prm.get_robot_state_for_vertex(q1),
            #                      "eef")
            if dist < dist_max:
                res.append((s, dist))
        res = sorted(res, key=lambda comp: comp[-1])
        # print res
        dists = []
        for r in res:
            fin_res.append(r[0])
            dists.append(r[1])
        return fin_res, dists

    def point_point_dist(self, prm, v1, v2):
        assert isinstance(prm, RoadMap)
        assert isinstance(v1, (int, Vertex))
        assert isinstance(v2, (int, Vertex))

        p1 = prm.get_eef_pose(v1, self.fk)
        p2 = prm.get_eef_pose(v2, self.fk)
        assert p1.pose_stamped[0].header.frame_id == p2.pose_stamped[0].header.frame_id
        dist = math.sqrt((p2.pose_stamped[0].pose.position.x - p1.pose_stamped[0].pose.position.x)**2 +
                         (p2.pose_stamped[0].pose.position.y - p1.pose_stamped[0].pose.position.y)**2 +
                         (p2.pose_stamped[0].pose.position.z - p1.pose_stamped[0].pose.position.z)**2
                         )
        return dist

    def connect_4(self, prm, q1, q2, only_transit=False):
        # type: (RoadMap, Vertex, Vertex) -> (bool, RobotTrajectory)
        assert isinstance(prm, RoadMap)
        if only_transit:
            type_q1 = prm.get_type_for_vertex(q1)
            type_q2 = prm.get_type_for_vertex(q2)
            if type_q1 != "travel" and type_q2 != "travel":
                return False, RobotTrajectory()

        q1_robot_state = prm.get_robot_state_for_vertex(q1)
        q2_robot_state = prm.get_robot_state_for_vertex(q2)

        mc = self.get_mgroup(prm.get_group_name())
        mc.set_start_state(q1_robot_state)

        plan = mc.plan(q2_robot_state.joint_state)

        if len(plan.joint_trajectory.points) >= 2:
            return True, plan
        else:
            return False, plan

    def connect_4_cart(self, prm, q1, q2):
        # type: (RoadMap, Vertex, Vertex) -> (bool, RobotTrajectory)
        assert isinstance(prm, RoadMap)

        # q1_robot_state = prm.get_robot_state_for_vertex(q1)
        pose_1 = prm.get_eef_pose(q1, fk=self.fk).pose_stamped[0].pose
        pose_2 = prm.get_eef_pose(q2, fk=self.fk).pose_stamped[0].pose
        # q2_robot_state = prm.get_robot_state_for_vertex(q2)

        mc = self.get_mgroup(prm.get_group_name())
        waypoints = [pose_1, pose_2]

        path, fraction = mc.compute_cartesian_path(waypoints, 0.05, 0.0)

        if fraction >= 0.9:
            return True, path
        else:
            return False, path

    def plan_dist(self, plan):
        assert isinstance(plan, RobotTrajectory)

        if len(plan.joint_trajectory.points) >= 2:
            jpt = plan.joint_trajectory.points[-1]  # type: JointTrajectoryPoint
            dist = jpt.time_from_start.to_sec()
            # print("dist: {}".format(dist))
            return True, plan, dist
        else:
            return False, plan, np.inf

    def mp_dist(self, prm, conf_1, conf_2, plan=None):
        '''
        This function uses moveit to plan a motion and returns the time needed for the planned motion.
        :param prm: Roadmap to retrieve the right move_group_name
        :param conf_1: starting configuration
        :param conf_2: goal configuration
        :param plan: if a plan is provided, only its length is calculated
        :return: success, plan, dist
        '''
        if plan is None:
            mc = self.get_mgroup(prm.get_group_name())
            mc.set_start_state(conf_1)

            plan = mc.plan(conf_2.joint_state)
        assert isinstance(plan, RobotTrajectory)

        if len(plan.joint_trajectory.points) >= 2:
            jpt = plan.joint_trajectory.points[-1]  # type: JointTrajectoryPoint
            dist = jpt.time_from_start.to_sec()
            return True, plan, dist
        else:
            return False, plan, np.inf

    def add_edge(self, prm, q1, q2, plan=None):
        # type: (RoadMap, Vertex, Vertex, RobotTrajectory) -> None
        if q1 is q2:
            return

        edge = prm.add_edge(q1, q2)

        if plan is None:
            connect, plan = self.connect_4(prm, q1, q2)
        assert isinstance(plan, RobotTrajectory)
        if plan is not None:
            jpt = plan.joint_trajectory.points[-1]  # type: JointTrajectoryPoint
            prm.set_edge_distance(edge, jpt.time_from_start.to_sec())
            prm.set_edge_traj(edge, plan)

        rospy.loginfo("Adding edge no {} with source {} and target {}".format(prm.get_number_of_edges(), q1, q2))
        return edge

    @staticmethod
    def same_component(prm, q1, q2):
        """
        method to check weather two nodes are connected in the roadmap.
        :param prm:
        :param q1: first node
        :param q2: second node
        :return: True if connected or identical, False if not yet connected
        """
        assert isinstance(prm, RoadMap)
        if int(q2) in prm.get_nodes_of_component(q1):
            return True
        else:
            return False

    def get_connection_distance(self, prm, q1, q2, cutoff=np.inf):
        if q1 is q2:
            return 0.0
        path = RoadmapUtils.find_shortest_path(prm, q1, q2)
        if len(path) == 0:
            print "Path len is 0!"
            return np.inf
        # assert len(path) != 0
        dist_min = self.calc_path_length(prm, path, cutoff=cutoff)
        return dist_min

    @staticmethod
    def find_shortest_path(prm, start, end):
        path = prm.find_path_prm(start, end)
        return path

    def calc_path_length(self, prm, path=[], cutoff=np.inf):
        assert len(path) != 0
        dist = 0
        for i in range(0, len(path) - 1, 1):
            dist += self.distance_for_nodes(prm, path[i], path[i + 1])
            if dist > cutoff:
                return np.inf
        return dist

    def distance_for_nodes(self, prm, v1, v2, dist_type="time"):
        conf_1 = prm.get_robot_state_for_vertex(v1)
        conf_2 = prm.get_robot_state_for_vertex(v2)
        dist = self.distance(prm, conf_1, conf_2, dist_type)
        return dist

    def distance(self, prm, conf_1, conf_2, type="ed", plan=None):
        # type: (RoadMap, RobotState, RobotState, str) -> float

        sq_sum = 0
        dist = 0
        if type is "point_dist":
            # names = self.robot_commander.get_joint_names(self.mgroup.get_name())
            names = prm.get_joint_names_of_group()
            for n in names:
                idx = conf_1.joint_state.name.index(n)
                sq_sum += np.square(conf_1.joint_state.position[idx] - conf_2.joint_state.position[idx])
            dist = np.sqrt(sq_sum)
        if type is "ed":
            # names = self.robot_commander.get_joint_names(self.mgroup.get_name())
            names = prm.get_joint_names_of_group()
            for n in names:
                idx = conf_1.joint_state.name.index(n)
                sq_sum += np.square(conf_1.joint_state.position[idx] - conf_2.joint_state.position[idx])
            dist = np.sqrt(sq_sum)
        elif type is "abs":
            names = prm.get_joint_names_of_group()
            for n in names:
                idx = conf_1.joint_state.name.index(n)
                val1 = conf_1.joint_state.position[idx]
                val2 = conf_2.joint_state.position[idx]
                sq_sum += np.abs(conf_1.joint_state.position[idx] - conf_2.joint_state.position[idx])
            dist = sq_sum
        elif type is "vec_abs":
            c1 = np.array(conf_1.joint_state.position)
            c2 = np.array(conf_2.joint_state.position)
            dist = np.sum(np.abs(c1 - c2))
            # diffs = numpy.array(numpy.abs(list(conf_1.robot_state.joint_state.position) - list(conf_2.robot_state.joint_state.position)))
            # dist = numpy.sum(diffs)
        elif type is "time":
            c1 = np.array(conf_1.joint_state.position)
            c2 = np.array(conf_2.joint_state.position)
            assert len(c1) == len(c2)
            dists = np.abs(c1 - c2)
            # dist = numpy.sum(numpy.abs(c1 - c2))
            # TODO: for nxo, the joint speeds are all 0.5. Quick hack!!!
            times = np.divide(dists, len(conf_1.joint_state.position) * [0.5])
            dist = np.max(times)
            # diffs = numpy.array(numpy.abs(list(conf_1.robot_state.joint_state.position) - list(conf_2.robot_state.joint_state.position)))
            # dist = numpy.sum(diffs)
        elif type is "mp":
            # use a motion planner to find time distance
            con, plan, dist = self.mp_dist(prm, conf_1, conf_2, plan=plan)
        return dist

    # TODO: test compute clash refactoring
    def amend_clash_for_prms(self, prm1, prm2, clash1, clash2, it=None, robot_info=None):
        # type: (RoadMap, RoadMap, dict[int, list[int]], dict[int, list[int]], None, RobotInfo) -> None
        sv = self.sv
        start_time = time.time()

        elapsed_time = time.time() - start_time

        # when method is called without iterator, we complete the whole clash with recursive calls
        if it is None:
            # find all vertices which are not in handled in clash
            not_in_clash_1 = set(prm1.vertices()).difference(clash1.keys())
            not_in_clash_2 = set(prm2.vertices()).difference(clash2.keys())
            if len(not_in_clash_1) > 0 or len(not_in_clash_2) > 0:
                if len(not_in_clash_1) > 0:
                    it = itertools.product(not_in_clash_1, prm2.vertices())
                    self.amend_clash_for_prms(prm1, prm2, clash1, clash2, it, robot_info)
                    self.amend_clash_for_prms(prm1, prm2, clash1, clash2, None, robot_info)
                    return
                if len(not_in_clash_2) > 0:
                    it = itertools.product(not_in_clash_2, prm1.vertices())
                    self.amend_clash_for_prms(prm2, prm1, clash2, clash1, it, robot_info)
                    return
            return

        print elapsed_time
        count = 0
        for t in it:  # type: tuple[Vertex, Vertex]
            rs = RoadmapUtils.build_robot_state(prm1, t[0], prm2, t[-1])
            if False:
                pub = rospy.Publisher("display_robot_state", DisplayRobotState, queue_size=10)
                # rospy.sleep(1.0)
                msg = DisplayRobotState()
                msg.state = rs
                # msg.state.joint_state.header = "blah"
                pub.publish(msg)
            gn = robot_info.getCompleteGroup()
            stateVal = sv.getStateValidity(rs, group_name=gn)
            # stateVal = self.SV_SRV.getStateValidity(rs, group_name="upperbody")

            # TODO: decide for a consistent semantics of stateval: Response vs bool - It's bool atm
            # if stateVal.valid:
            if stateVal is True:
                try:
                    clash1[int(t[0])].add(int(t[-1]))
                except KeyError:
                    clash1[int(t[0])] = set([int(t[-1])])
                try:
                    clash2[int(t[-1])].add(int(t[0]))
                except KeyError:
                    clash2[int(t[-1])] = set([int(t[0])])

            count += 1
            if count > 1000000:
                break
            if count % 1000 == 0:
                print count


            elapsed_time = time.time() - start_time
            # print elapsed_time

    @staticmethod
    def build_robot_state(prm_1, v1, prm_2, v2):
        # type: (RoadMap, Vertex, RoadMap, Vertex) -> RobotState

        # rs = RobotState()

        rs1 = prm_1.get_robot_state_for_vertex(v1)  # type: RobotState
        rs2 = prm_2.get_robot_state_for_vertex(v2)  # type: RobotState

        joint_names_2 = prm_2.get_joint_names_of_group()

        rs = rs1
        joint_state_pos_lst = list(rs.joint_state.position)

        for joint_name in joint_names_2:
            i_prm = rs2.joint_state.name.index(joint_name)
            i_rs = rs.joint_state.name.index(joint_name)
            joint_state_pos_lst[i_rs] = rs2.joint_state.position[i_prm]

        rs.joint_state.position = tuple(joint_state_pos_lst)

        return rs