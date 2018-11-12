#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import copy
import itertools

import rospy
from roadmap_planner.service_proxies import PlannerServiceProxies

from geometry_msgs.msg import TransformStamped, PoseStamped

from roadmap_planning_common_msgs.srv import AddObjectRequest, AddOvcCtRequest, AddOVCRequest, AddFrameRequest, \
    AddPoseRequest
from roadmap_planning_common_msgs.msg import OrderedVisitingConstraint, ConstraintType, InterOVCConstraint

from rospy_message_converter import json_message_converter



class PlannerInputManager:
    '''
    This class stores and manages the inputs for the STAAMS Solver. Add your locations, OVCs, and constraints. You can
    dump and parse a json version of these inputs. The PlannerInputManager makes also sure to transfer your
    specifications to the solver and trigger the Solver.
    '''
    def __init__(self):
        self._ovcs = {}
        self._locs = {}
        self._cts = {}
        self._frames = {}

    def add_loc(self, poses, names, types):
        '''
        This function adds a set of locations to the task template
        :param poses: A list of PoseStamped objects in any reference frame you added to the scene graph
        :param names: List of names for the poses. They should be unique. If not, we generate a name.
        :param types: List of types for the locations.
        :return: A list of names as they were registered and a list of ints encoding some tests. If a name had to be
        made unique, it will be reflected here.
        '''
        # pack inputs into lists if only a single loc should be added.
        if type(types) is str:
            types = [types]
        if type(poses) is PoseStamped:
            poses = [poses]
        if type(names) is str:
            names = [names]

        requests = []
        result = []
        assert len(poses) == len(names) == len(types)
        for pose, name, pose_type in zip(poses, names, types):
            req = AddPoseRequest()
            req.pose = pose
            req.loc_name = name
            req.loc_type = pose_type
            requests.append(req)

            if req.loc_name in self._locs.keys():
                rospy.logwarn("location {} is already present. We will skip it. Use a unique name.".format(req.loc_name))
                # rospy.logwarn("location {} is already present. We will rename the new loc.")
                result.append(0)
            else:
                self._locs[name] = req
                result.append(1)

        return [r.loc_name for r in requests], requests, result

    def add_ovc(self, OVCs):
        # type; (List[OrderedVisitingConstraint]) -> Dict[str, OrderedVisitingConstraint]

        if type(OVCs) is not list:
            assert type(OVCs) is OrderedVisitingConstraint
            OVCs = [OVCs]

        for ovc in OVCs:  # type: OrderedVisitingConstraint
            req = AddOVCRequest()
            req.ovcs.append(ovc)

            self._ovcs[ovc.name] = req

        return self._ovcs

    def add_ovc_ct(self, constraint_type, first_ovcs, second_ovcs=None, interval=None, param=None):
        # type: (ConstraintType, list[str], list[str], list[int], list[int]) -> list[AddOvcCtRequest]

        # print(ct)
        #
        # for first_ovc in ct[1]:
        #     ovc_msg = OrderedVisitingConstraint()
        #     try:
        #         ovc_msg.name = ovc_dict[first_ovc][0]
        #     except TypeError:
        #         continue
        #     ct_ovc_list = [ovc_msg]
        #
        #     for second_ovc in ct[2]:
        #         ovc_msg = OrderedVisitingConstraint()
        #         try:
        #             ovc_msg.name = ovc_dict[second_ovc][0]
        #         except TypeError:
        #             continue
        #         ct_ovc_list.append(ovc_msg)
        #
        #     intervals = [-1] * len(ct_ovc_list)
        #     # comment out for sc 1
        #     PlannerServiceProxies.add_ovc_ct(ovc=ct_ovc_list, interval=intervals, ct_type=ConstraintType.StartsAfterEnd)


        '''
            roadmap_planning_common_msgs/ConstraintType ct_type
            roadmap_planning_common_msgs/OrderedVisitingConstraint[] ovc
            int32[] interval
            int32[] params
        '''
        requests = []

        if constraint_type is ConstraintType.StartsAfterEnd:
            it = itertools.product(first_ovcs, second_ovcs)
            for ovc_1, ovc_2 in it:
                req = AddOvcCtRequest()
                req.ct.ct_type.val = constraint_type
                req.ct.ovc_name.append(ovc_1)
                req.ct.ovc_name.append(ovc_2)
                req.ct.ovc_interval = [-1] * len(req.ct.ovc_name)

                requests.append(req)

                # TODO: should constraints also be named?

        self._cts += requests

        return requests

    def add_ovc_ct_direct(self, ct):
        # type: (InterOVCConstraint) -> list[AddOvcCtRequest]


        requests = []

        req = AddOvcCtRequest()
        req.ct = ct
        requests.append(req)

        # TODO: should constraints also be named?

        self._cts[req.ct.name] = req

        return requests

    def add_frame(self, transform):
        # type: (TransformStamped) -> str

        req = AddFrameRequest()
        req.transform = transform
        print(req.transform.header.frame_id)
        print(req.transform.child_frame_id)

        self._frames[req.transform.child_frame_id] = req

        return req.transform.child_frame_id


    def read_planner_input_file(self):

            json_strs = []
            with open('scene_glue_01.json', 'r') as text_file:
                json_strs = text_file.readlines()

            for json in json_strs:
                if json == "\n":
                    continue
                req = json_message_converter.convert_json_to_ros_message(
                    'roadmap_planning_common_msgs/AddObjectRequest', json)

    def write_planner_input_file(self, file_name):

        s = ""

        for obj in self._frames.values():
            json_str = json_message_converter.convert_ros_message_to_json(obj)
            s += json_str + "\n"

        s += "\n"

        for obj in self._locs.values():
            json_str = json_message_converter.convert_ros_message_to_json(obj)
            s += json_str + "\n"

        s += "\n"

        for obj in self._ovcs.values():
            json_str = json_message_converter.convert_ros_message_to_json(obj)
            s += json_str + "\n"

        s += "\n"

        for obj in self._cts.values():
            json_str = json_message_converter.convert_ros_message_to_json(obj)
            s += json_str + "\n"

        s += "\n"

        print(s)

        with open(file_name + '.json', 'w') as text_file:
            text_file.write(s)


