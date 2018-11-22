#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import os
import copy
import itertools

from ortools.constraint_solver import pywrapcp

from roadmap_tools.SceneObjectSearchMapping import SceneObjectSearchMapping
# from roadmap_planner.prm_planner import RoadmapPlanner
from roadmap_planner.ovc_planner import OvcPlanner as RoadmapPlanner
from roadmap_tools.roadmap_clash import RoadMapClash
from roadmap_tools.prm_factory import RoadMapFactory
from roadmap_tools.prm import RoadMap
from moveit_msgs.msg import RobotState

import tf2_ros
import tf2_geometry_msgs

import rospy
import sys

import time

from roadmap_planner.ovc_exp import ovc_experiment
import cPickle as pickle

from roadmap_planner.ordered_visiting_ct_var import OrderedVisitingConstraintVar

from roadmap_planning_common_msgs.srv import StringQueryResponse, StringQuery, StringQueryRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from roadmap_planning_common_msgs.srv import AddOVC, AddOVCRequest, AddOVCResponse, \
    AddOvcCt, AddOvcCtRequest, AddOvcCtResponse, SetState, SetStateRequest, SetStateResponse, GetIntListForStringList, GetIntListForStringListRequest, GetIntListForStringListResponse, AddObject, AddObjectRequest, AddObjectResponse
from roadmap_planning_common_msgs.msg import ConstraintType

from roadmap_tools.roadmap_util import RoadmapUtils

from roadmap_tools.rs_vertex import rs_vertex
from roadmap_tools.robot_info import RobotInfo


class RoadmapPlannerNode:

    def __init__(self, node_name="prm_planner_wrapper", robot_info=None):
        self._OVC = {}
        self._next_ovc_id = 0
        self.rp = None  # type: RoadmapPlanner

        parameter_ns = "SolverSetup"

        # self.exp_obj = ovc_experiment(name="created_in_subprocess")
        self.exp_obj = None
        # self.exp_host = "/Kuka_glue_exp/"
        self.exp_host = parameter_ns
        # exp = Experiment("test run with standard configuration")

        self.robot_name = rospy.get_param(self.exp_host + '/ROBOT_NAME')

        robot_info = RobotInfo.getRobotInfo()

        self.robot_info = robot_info  # type: RobotInfo

        self.roadmap_util = RoadmapUtils(fk=self.robot_info.getFK(), ik=self.robot_info.getIK(), sv=self.robot_info.getSV())

        self.ik_seed_rs = rs_vertex.load_off_joint_state(robot_name=self.robot_name,
                                                         file_name_suffix="ik_seed")  # type: RobotState
        self.off_rs = rs_vertex.load_off_joint_state(robot_name=self.robot_name, file_name_suffix="off")  # type: RobotState

        self.STEPS_MAX = rospy.get_param(self.exp_host + '/STEPS_MAX')
        print "Steps_max: {}".format(self.STEPS_MAX)
        self.TIME_MAX = rospy.get_param(self.exp_host + '/TIME_MAX')
        self.LUBY_RESTART_CONSTANT = rospy.get_param(self.exp_host + '/LUBY_RESTART_CONSTANT')
        self.SEED = rospy.get_param(self.exp_host + '/SEED')

        self.group_names = rospy.get_param(self.exp_host + '/GROUP_NAMES')
        self.roadmap_names = rospy.get_param(self.exp_host + '/ROADMAP_NAMES')
        self.clash_name = rospy.get_param(self.exp_host + '/CLASH_NAME')

        if self.group_names is None:
            self.group_names = rospy.get_param(self.exp_host + 'setup/group_names', ["left_arm", "right_arm"])
        if self.roadmap_names is None:
            self.roadmap_names = rospy.get_param(self.exp_host + 'setup/roadmap_names',
                                            {'right_arm': 'prm_right_arm_2018-01-08 16:47:45.960621',
                                             'left_arm': 'prm_left_arm_2018-01-08 16:46:32.374956'})
        print "Roadmap names: {}".format(self.roadmap_names)
        if self.clash_name is None:
            self.clash_name = rospy.get_param(self.exp_host + 'setup/clash_name',
                                         "clash_2018-01-08 16:51:09.845139")
        print "clash name: {}".format(self.clash_name)

        self.prepare_planner()

        # ROS related stuff
        rospy.init_node(node_name)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.srv_add_roadmap = rospy.Service(node_name + "/add_roadmap", StringQuery, self.handle_add_roadmap)
        self.srv_add_clash = rospy.Service(node_name + "/add_clash", StringQuery, self.handle_add_clash)

        self.srv_add_ovc = rospy.Service(node_name + "/add_ovc", AddOVC, self.handle_add_ovc)

        self.srv_add_ovc_ct = rospy.Service(node_name + "/add_ovc_ct", AddOvcCt, self.handle_add_ovc_ct)

        self.srv_set_state = rospy.Service(node_name + "/set_state", SetState, self.handle_set_state)

        self.srv_solve = rospy.Service(node_name + "/solve", Empty, self.handle_solve)

        self.srv_save_prm_data = rospy.Service(node_name + "/save_prm_data", Empty, self.handle_save_prm_data)

        self.srv_reset_planner = rospy.Service(node_name + "/reset_planner", Empty, self.handle_reset_planner)

        self.srv_get_loc_for_names = rospy.Service(node_name + "/get_locs_for_names", GetIntListForStringList, self.handle_get_locations_for_names)

        self.srv_add_object_online = rospy.Service(node_name + "/add_object_online", AddObject,
                                                   self.handle_add_pose_online)

        self.srv_print_data = rospy.Service(node_name + "/print_data", Empty, self.handle_print_data)

        rospy.spin()

    def handle_reset_planner(self, req):
        # type: (EmptyRequest) -> EmptyResponse
        res = EmptyResponse()

        self.reset_planner()

        return res

    def handle_get_locations_for_names(self, req):
        # type: (GetIntListForStringListRequest) -> GetIntListForStringListResponse
        res = GetIntListForStringListResponse()

        res.int_out = self.get_locations_for_names(req.str_in)
        return res


    def get_locations_for_names(self, names=[]):
        locations = []
        name_mapping = self.rp.sosm.get_name_int_alias()

        for name in names:
            try:
                locations.append(name_mapping[name])
            except KeyError:
                rospy.logerr("No location for object {} known.".format(name))

        return locations

    def handle_add_pose_online(self, req):
        # type: (AddObjectRequest) -> AddObjectResponse
        # TODO: Make MAX_CONNECIONS a parameter
        MAX_CONNECTIONS = 6
        res = AddObjectResponse()
        res.success = False

        if req.object_name != "travel" and len(self.get_locations_for_names([req.object_name])) > 0:
            rospy.loginfo("A location with name {} is already registered in roadmaps".format(req.object_name))
            return res

        ct_open_list = []

        for rm_name, rm in self.rp.roadmaps.items():

            node_success = False
            nr_connections = 0
            # find data: rs, name, typ
            # add node to rm
            vert = self.add_pose_to_roadmaps(name=req.object_name, group=rm_name, pose=req.pose, obj_type=req.object_type)
            if vert is None:
                continue
            # connect node in rm
            nh, dists = self.roadmap_util.get_neighborhood(rm, vert, dist_max=0.3)
            for index, v2 in enumerate(nh):
                if nr_connections >= MAX_CONNECTIONS:
                    break
                if not self.roadmap_util.same_component(rm, vert, v2):
                    connect, plan = self.roadmap_util.connect_4(prm=rm, q1=vert, q2=v2, only_transit=True)
                    if connect:
                        if self.roadmap_util.plan_dist(plan)[2] < 3.0:
                            self.roadmap_util.add_edge(rm, vert, v2, plan)
                            nr_connections += 1
                            node_success = True
                            res.success = True

                elif self.roadmap_util.distance_for_nodes(rm, vert, v2, "mp") * 1.3 < self.roadmap_util.get_connection_distance(rm, vert, v2):
                    connect, plan = self.roadmap_util.connect_4(prm=rm, q1=vert, q2=v2, only_transit=True)
                    if connect:
                        if self.roadmap_util.plan_dist(plan)[2] < 3.0:
                            self.roadmap_util.add_edge(rm, vert, v2, plan)
                            nr_connections += 1
                            node_success = True
                            res.success = True

            if rm.get_number_of_vertices() < 100:
                res.success = True
            if not node_success and not res.success:
                # TODO: transfer to Roadmap class
                rm._g.remove_vertex(vert)
                continue

            # fill open-list for collision checks
            # it = itertools.product([vert])
            for key in self.rp.clash.clashes.keys():
                if key[0] == rm_name:
                    it = itertools.product([vert], self.rp.roadmaps[key[-1]].vertices())
                    self.roadmap_util.amend_clash_for_prms(self.rp.roadmaps[key[0]], self.rp.roadmaps[key[-1]], self.rp.clash.clashes[key], self.rp.clash.clashes[(key[-1], key[0])], it, self.robot_info)
            # make collision checks and amend collision tables
        if res.success:
            self.update_sosm()

        for rm_name, rm in self.rp.roadmaps.items():
            print(rm_name + " has size: {}".format(rm.get_number_of_vertices()))
            print(rm_name + " has number of edges: {}".format(rm.get_number_of_edges()))

        return res


    def add_pose_to_roadmaps(self, name="gluepoint_12", group="r1_arm", pose=None, obj_type=None):

        ee_frame = self.robot_info.getEndEffector(group=group)
        base_frame = self.roadmap_util.get_mgroup(group).get_pose_reference_frame().replace("/", "")
        # ee2tip = self.robot_info.get_eef2tip_transform(group, now=rospy.Time.now())  # type: TransformStamped
        self.ik_seed_rs = rs_vertex.load_off_joint_state(robot_name=self.robot_name,
                                                         file_name_suffix="ik_seed")  # type: RobotState
        self.off_rs = rs_vertex.load_off_joint_state(robot_name=self.robot_name, file_name_suffix="off")  # type: RobotState

        rs = copy.copy(self.ik_seed_rs)  # type: RobotState

        active_joints = self.roadmap_util.get_mgroup(group).get_active_joints()
        joint_state_list = list(rs.joint_state.position)

        for idx, jn in enumerate(rs.joint_state.name):
            if jn not in active_joints:
                idx_2 = self.off_rs.joint_state.name.index(jn)
                joint_state_list[idx] = self.off_rs.joint_state.position[idx_2]

        rs.joint_state.position = tuple(joint_state_list)

        # transform = self.tf_buffer.lookup_transform("world",
        # transform = self.tf_buffer.lookup_transform("WAIST",
        transform = self.tf_buffer.lookup_transform(base_frame,
                                               pose.header.frame_id,  # source frame
                                               rospy.Time(0),  # get the tf at first available time
                                               rospy.Duration(1.0))  # wait for 1 second

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
        # pose_transformed_2 = tf2_geometry_msgs.do_transform_pose(pose_transformed, ee2tip)
        #
        # ######################### get transformation matrices#########################
        #
        # from tf import TransformerROS, transformations
        # import numpy as np
        # self.tf_ros = TransformerROS()
        #
        # ref = pose_transformed
        # ref_trans = self.tf_ros.fromTranslationRotation((ref.pose.position.x,
        #                                                  ref.pose.position.y,
        #                                                  ref.pose.position.z),
        #                                                 (ref.pose.orientation.x,
        #                                                  ref.pose.orientation.y,
        #                                                  ref.pose.orientation.z,
        #                                                  ref.pose.orientation.w,))
        #
        # ee2tip_inv = np.linalg.inv(self.tf_ros.fromTranslationRotation((ee2tip.transform.translation.x,
        #                                                  ee2tip.transform.translation.y,
        #                                                  ee2tip.transform.translation.z),
        #                                                  (ee2tip.transform.rotation.x,
        #                                                   ee2tip.transform.rotation.y,
        #                                                   ee2tip.transform.rotation.z,
        #                                                   ee2tip.transform.rotation.w)))
        #
        # # ref_trans_inv = np.linalg.inv(ref_trans)
        #
        # new_trans = np.matmul(ref_trans, ee2tip_inv)
        # new_trans = np.matmul(ee2tip_inv, ref_trans)
        #
        #
        # trans = transformations.translation_from_matrix(new_trans)
        # rot = transformations.quaternion_from_matrix(new_trans)
        #
        # pose_trans_ee = tf2_geometry_msgs.PoseStamped()
        # pose_trans_ee.header.frame_id = pose_transformed.header.frame_id
        # pose_trans_ee.pose.position.x = trans[0]
        # pose_trans_ee.pose.position.y = trans[1]
        # pose_trans_ee.pose.position.z = trans[2]
        # pose_trans_ee.pose.orientation.x = rot[0]
        # pose_trans_ee.pose.orientation.y = rot[1]
        # pose_trans_ee.pose.orientation.z = rot[2]
        # pose_trans_ee.pose.orientation.w = rot[3]

        resp = self.roadmap_util.ik.getIK(group, self.robot_info.getEndEffector(group), pose_transformed, robot_state=rs, avoid_collisions=True)
        if resp.error_code.val == 1:
            rs = RobotState()
            rs.joint_state = resp.solution.joint_state
            obj_name = name

            vert = self.rp.roadmaps[group].add_vertex(robot_state=rs, name=obj_name, obj_type=obj_type)

            return vert
        return None

    def update_sosm(self):
        old_sosm = self.rp.sosm
        sosm = SceneObjectSearchMapping(self.rp.roadmaps.values(), old_sosm)
        self.rp.sosm = sosm

    def prepare_planner(self):
        # prepare roadmap_planner

        self.exp_obj = ovc_experiment(name="created_in_subprocess")
        self.rp = RoadmapPlanner()  # type: RoadmapPlanner
        for gn in self.group_names:
            assert gn in self.roadmap_names.keys()
            if self.roadmap_names[gn] == 'empty':
                rm = RoadMap(self.roadmap_util.get_mgroup(name=gn), group_name=gn)
                self.rp.roadmaps[gn] = rm
                rospy.loginfo("Loaded empty roadmap {} into RoadmapPlanner.".format(rm.get_fingerprint()))

            else:
                self.rp.roadmaps[gn] = RoadMapFactory.load_prm_from_database(self.roadmap_names[gn])

        if self.clash_name == 'empty':
            rmc = RoadMapClash(self.group_names)
            for group_name in self.group_names:
                r = self.rp.roadmaps[group_name]
                    # 'RoadMapFactory.load_prm_from_database(roadmap_names[group_name])
                rmc.add_roadmap(r)

            rmc.compute_clash_for_prms(sv=self.roadmap_util.sv)
            clash_name = "clash_" + str(time.time()).split(".")[0]
            rmc.save_clash(groups=self.group_names, file_name=clash_name)
            self.rp.clash = rmc
        else:
            self.rp.clash.load_clash(groups=self.group_names, file_name=self.clash_name)

        # complete the clashes for all combinations of roadmaps
        it = itertools.combinations(self.rp.roadmaps.values(), 2)
        for rm1, rm2 in it:
            clash_1 = self.rp.clash.get_clash(rm1.get_group_name(), rm2.get_group_name())
            clash_2 = self.rp.clash.get_clash(rm2.get_group_name(), rm1.get_group_name())
            self.roadmap_util.amend_clash_for_prms(rm1, rm2, clash_1, clash_2, it=None, robot_info=self.robot_info)

        self.rp.clash.verify_clash_roadmap_combinations(self.rp.roadmaps)

        sosm = SceneObjectSearchMapping(self.rp.roadmaps.values())
        self.rp.sosm = sosm

        self.rp.build_manipulation_model(self.STEPS_MAX, self.TIME_MAX,
                                         collision_on=rospy.get_param(self.exp_host + '/COLLISION', default=True))

        self._OVC = {}
        self._next_ovc_id = 0

    def reset_planner(self):
        self.prepare_planner()

    def handle_save_prm_data(self, req):
        res = EmptyResponse()
        for rm in self.rp.roadmaps.values():
            RoadMapFactory.save_prm_to_database(rm, replace=False, suffix="saved")
            rospy.loginfo("Saved roadmap {} to database".format(rm.get_fingerprint()))

        self.rp.clash.save_clash(groups=self.group_names, file_name="clash_{}".format(time.time()))

        return res

    def handle_print_data(self, req):
        res = EmptyResponse()
        s = ""
        s += "Clashes: \n"
        s += str(self.rp.clash.clashes)
        s += "\n"

        print(s)

        return res

    def handle_set_state(self, req):
        # type: (SetStateRequest) -> SetStateResponse
        res = SetStateResponse()

        res.success = self.set_state(req.state_index, req.group, req.values.values, req.use_location)
        return res

    def set_state(self, state_index=0, group="r1_arm", state=[0], use_location=False):
        '''
        This service sets a state of a group indicated by state_index to state. Used to set start and goal state
        :param state_index: Index of selected state (has to be in the range of created variables. 0 for initial state, -1 for final state
        :param group: string indicating the group
        :param state: conf index from the roadmap
        :return: success
        '''

        if group not in self.rp.roadmaps.keys():
            return False

        if use_location:
            loc2node = self.rp.sosm.get_alias_to_poses()
            my_state = [loc2node[group][s] for s in state]
            self.rp.add_visiting_Ct(group=group, locations=my_state, state_index=state_index)
        else:
            self.rp.add_visiting_Ct(group=group, locations=state, state_index=state_index)
        return True

    def handle_add_ovc_ct(self, req):
        # type: (AddOvcCtRequest) -> AddOvcCtResponse
        res = AddOvcCtResponse()

        ovc_names = []
        for ovc_name in req.ct.ovc_name:
            try:
                name = int(ovc_name)
                ovc_names.append(name)
            except ValueError:
                rospy.logerr("Got invalid name {}. Only Strings convertible to ints are allowed.".format(OVC.name))
                res.success = False
                res.msg = "Got invalid name {}. Only Strings convertible to ints are allowed.".format(OVC.name)
                return res

        # check if all names are in the dictionary keys
        for name in ovc_names:
            if name not in self._OVC.keys():
                rospy.logerr("Cannot add constraint regarding OVC {}, because it doesn't exist.".format(name))
                res.success = False
                res.msg = "Cannot add constraint regarding OVC {}, because it doesn't exist.".format(name)
                return res

        assert len(ovc_names) == len(req.ct.ovc_interval)
        intervals = []
        ovc_list = []
        for name, interval in zip(ovc_names, req.ct.ovc_interval):
            intervals.append(self._OVC[name].getInterval(interval))
            ovc_list.append(self._OVC[name])

        ###############################################################################
        # the first interval in intervals must start after all remaining intervals
        if req.ct.ct_type.val == ConstraintType.StartsAfterEnd:
            solver = self.rp.solver
            if len(intervals) < 2:
                res.success = False
                res.msg = "I got less than 2 intervals for an at least binary constraint."
                return res

            pri_int = intervals[0]  # type: pywrapcp.IntervalVar
            pri_ovc = ovc_list[0]  # type: OrderedVisitingConstraintVar
            pri_int_ind = req.ct.ovc_interval[0]
            for sec_int, sec_ovc, sec_int_ind in zip(intervals[1:], ovc_list[1:], req.ct.ovc_interval[1:]):  # type: (pywrapcp.IntervalVar, OrderedVisitingConstraintVar, int)
                # ct = pri_int.StartsAfterEnd(sec_int)  # type: pywrapcp.Constraint
                ct = sec_int.StartsAfterEnd(pri_int)
                check_res = solver.CheckConstraint(ct)
                rospy.loginfo("Checked constraint {} with result: {}.".format(ct, check_res))
                solver.Add(ct)
                rospy.loginfo("Added Constraint {} to the solver.".format(ct))

                # adding dependencies between OVCs to be considered in the variable selection heuristic
                self.rp.dependencies.addDependency(
                    sec_ovc._conf_connect_vars[sec_int_ind],
                    pri_ovc._conf_connect_vars[pri_int_ind],
                    dependency_type=1
                )

                from roadmap_planner.CustomConstraints import CondOVCMonotonicCt
                ct = CondOVCMonotonicCt(solver, [pri_ovc, sec_ovc])
                check_res = solver.CheckConstraint(ct)
                rospy.loginfo("Checked constraint {} with result: {}.".format(ct, check_res))
                solver.Add(ct)
                rospy.loginfo("Added Constraint {} to the solver.".format(ct))

            res.success = True
            res.msg = "Added {} constraints to the solver.".format(len(intervals) - 1)

        return res

    def handle_add_ovc(self, req):
        # type: (AddOVCRequest) -> AddOVCResponse
        res = AddOVCResponse()
        ovcs = req.ovcs

        ovc_list = []
        for ovc in ovcs:
            domains = [list(dom.values) for dom in ovc.location_domains]
            ovc_list.append(self.rp.addFreeOVC(ovc.groups, domains, ranges=ovc.duration_ranges))

        # put the ovcs to the list for later reference
        res.names = [str(integer) for integer in self.persist_ovc(ovc_list)]

        res.success = True
        # res.names = [ovc.get_name() for ovc in ovc_list]
        return res

    def persist_ovc(self, ovcs=[]):
        '''
        This function adds the Ordered Visiting Constraints given in ovcs to a dictionary in this class and returns
        their names for later reference.
        :param ovcs: Instance of OrderedVisitingConstraintVar or list of OrderedVisitingConstraintVar
        :return: list of integers (keys of given OVCs in the dictionary
        '''
        ovc_names = []

        # if only a single OVC is given, we add it.
        if isinstance(ovcs, OrderedVisitingConstraintVar):
            self._OVC[self._next_ovc_id] = ovcs
            ovc_names.append(self._next_ovc_id)
            self._next_ovc_id += 1
            return ovc_names

        # if a list of OVCs is given, we test each and add them
        if isinstance(ovc_names, list):
            for ovc in ovcs:
                assert isinstance(ovc, OrderedVisitingConstraintVar)
                self._OVC[self._next_ovc_id] = ovc
                ovc_names.append(self._next_ovc_id)
                self._next_ovc_id += 1

            return ovc_names

    def print_conf_state(self):
        rospy.loginfo(self.rp.solver)
        rospy.loginfo(self.rp.roadmaps)
        rospy.loginfo(self.rp.clash)
        rospy.loginfo(self.rp.check_planning_requirements())

        rospy.loginfo(self.rp.print_model())

    def handle_add_clash(self, req):
        # type: (StringQueryRequest) -> StringQueryResponse
        res = StringQueryResponse()
        res.success = False
        # TODO: implement add clash
        input = req.input
        clash_name = input[0]

        return res

    def handle_add_roadmap(self, req):
        # type: (StringQueryRequest) -> StringQueryResponse
        res = StringQueryResponse()
        res.success = False

        input = req.input
        group = input[0]
        roadmap_name = [1]

        if roadmap_name == "empty":
            rm = RoadMap(self.roadmap_util.get_mgroup(name=group), group_name=group)
            self.rp.roadmaps[group] = rm
            res.success = True
            rospy.loginfo("Loaded empty roadmap {} into RoadmapPlanner.".format(rm.get_fingerprint()))

        else:
            rm = RoadMapFactory.load_prm_from_database(roadmap_name)
            if isinstance(rm, RoadMap) and rm.get_group_name() == group:
                self.rp.roadmaps[group] = rm
                res.success = True
                rospy.loginfo("Loaded roadmap {} into RoadmapPlanner.".format(rm.get_fingerprint()))

        self.print_conf_state()
        return res

    def handle_solve(self, req):
        res = EmptyResponse()
        rospy.loginfo("Start solving problem.")

        self.rp.build_constraints()

        self.rp.define_decision_builders()
        self.rp.define_search_monitors()

        self.rp.print_model()

        time_limit = rospy.get_param(self.exp_host + '/solver_time', default=10000)
        EXECUTE_MOTION = rospy.get_param(self.exp_host + '/EXECUTE_MOTION', default=False)

        par = rospy.get_param(self.exp_host + '')
        self.exp_obj.save_exp_setup(par)

        rospy.loginfo("Solver Settings are: {}.".format(par))

        # results = rp.manual_global_search(time_limit=40000)
        results = self.rp.ovc_search(time_limit=time_limit, seed=self.SEED,
                                     luby_constant=self.LUBY_RESTART_CONSTANT,
                                     bc_solution=self.exp_obj.add_solution,
                                     bc_solver_stats=self.exp_obj.add_solver_stats,
                                     bc_extra_sol_files=self.exp_obj.add_extra_sol_file,
                                     bc_motion_req=self.exp_obj.add_motion_requests,
                                     exp=self.exp_obj,
                                     EXECUTE_MOTION=EXECUTE_MOTION)

        self.save_to_file(self.exp_obj)

        for gn, rm in self.rp.roadmaps.items():
            self.save_to_file(rm, gn)

        self.save_to_file(self.rp.sosm)
        self.save_to_file(self.rp.clash)

        print self.exp_obj.destination_folder

        return res

    def save_to_file(self, obj, name=""):
        path_to_file = None
        if isinstance(obj, RoadMap):
            path_to_file = os.path.join(self.exp_obj.destination_folder, "data_files/roadmaps/prm_{}.pkl".format(name))
        elif isinstance(obj, RoadMapClash):
            path_to_file = os.path.join(self.exp_obj.destination_folder, "data_files/clashes/clash.pkl")
        elif isinstance(obj, SceneObjectSearchMapping):
            path_to_file = os.path.join(self.exp_obj.destination_folder, "data_files/sosm.pkl")
        elif isinstance(obj, ovc_experiment):
            path_to_file = os.path.join(self.exp_obj.destination_folder, "exp.pkl")

        if not path_to_file:
            return False

        # create folder if necessary
        if not os.path.exists((os.path.dirname(path_to_file))):
            os.makedirs((os.path.dirname(path_to_file)))

        with open(path_to_file, 'wb') as output:
            pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)
        return True


if __name__ == "__main__":
    sys.setrecursionlimit(1000000)

    robot_name = rospy.get_param("SolverSetup/ROBOT_NAME", default=None)
    rospy.loginfo("Found robot_name {} on the parameter server".format(robot_name))

    if len(sys.argv) == 2:
        print("usage: prm_planner_node.py robot_name")
        print("robot_name is {}".format(sys.argv[1]))
        myargv = rospy.myargv(argv=sys.argv)
        rospy.set_param("SolverSetup/ROBOT_NAME", myargv[1])

    if len(sys.argv) == 3:
        rospy.loginfo("usage: prm_planner_node.py robot_name robot_info_class")
        rospy.loginfo("robot_name is {}".format(sys.argv[1]))
        rospy.loginfo("robot_info_class is {}".format(sys.argv[2]))

        myargv = rospy.myargv(argv=sys.argv)
        rospy.loginfo(myargv)
        rospy.loginfo(sys.argv)
        rospy.set_param("SolverSetup/ROBOT_NAME", myargv[1])
        rospy.set_param("SolverSetup/ROBOT_INFO_CLASS", myargv[2])

    node = RoadmapPlannerNode()




