#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import rospy
from roadmap_planning_common_msgs.srv import AddOVC, AddOVCResponse, AddOVCRequest, AddOvcCt, AddOvcCtResponse, \
    AddOvcCtRequest, SetState, SetStateResponse, SetStateRequest, GetIntListForStringList, \
    GetIntListForStringListResponse, GetIntListForStringListRequest, AddObject, AddObjectResponse, AddObjectRequest, \
    StringQueryRequest, StringQuery
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from roadmap_planning_common_msgs.msg import ovc, Range, Domain, OrderedVisitingConstraint, ConstraintType


class PlannerServiceProxies:

    @staticmethod
    def add_ovc(groups=["r1_arm", "r2_arm"], domains=[[1], [5]], location_names=None, ranges=[[30, 500], [30, 500]]):
        '''
        Function that reates an addOvcRequest and sends it to the prm_planner_wrapper
        :param groups: list of groups, that should be considered for executing the OVC
        :param domains: list of int-lists with location indexes (you can use the SceneObjectSearchMapping to retrieve them)
        :param ranges: list (len == len(domains) of int-lists (len == 2)
        :return: the name of the created OVC (in a list)
        '''
        rospy.wait_for_service("/prm_planner_wrapper/add_ovc")
        srv_add_ovc = rospy.ServiceProxy("/prm_planner_wrapper/add_ovc", AddOVC)

        ovc_req = AddOVCRequest()

        o = OrderedVisitingConstraint()
        o.groups = groups

        if location_names is not None:
            pass
            #TODO: implement!!!

        assert len(domains) == len(ranges)

        for d in domains:
            dom = Domain()
            dom.values = d
            o.location_domains.append(dom)

        for ra in ranges:
            if isinstance(ra, Range):
                o.duration_ranges.append(ra)
            elif isinstance(ra, list):
                assert len(ra) == 2
                ran = Range()  # type: range
                ran.min = ra[0]
                ran.max = ra[1]
                o.duration_ranges.append(ran)

        ovc_req.ovcs.append(o)
        res = srv_add_ovc.call(ovc_req)  # type: AddOVCResponse
        print(res.names)
        return res.names

    @staticmethod
    def get_locs_for_names(names=[]):
        '''
         This Function queries the location aliases for a list of names in Roadmap
         :param names: list of object name strings
         :return: list of ints - the location aliases
         '''
        service_name = "/prm_planner_wrapper/get_locs_for_names"
        srv_get_locs_for_names = rospy.ServiceProxy(service_name, GetIntListForStringList)

        req = GetIntListForStringListRequest()
        req.str_in = names
        res = srv_get_locs_for_names.call(req)  # type: GetIntListForStringListResponse

        return [loc for loc in res.int_out]

    @staticmethod
    def add_object_online_client(pose, name, obj_type):
        service_name = "/prm_planner_wrapper/add_object_online"
        srv_add_obj_online = rospy.ServiceProxy(service_name, AddObject)

        req = AddObjectRequest()
        req.pose = pose
        req.object_name = name
        req.object_type = obj_type

        res = srv_add_obj_online.call(req)  # type: AddObjectResponse

        return res.success

    @staticmethod
    def solve_client():
        service_name = "/prm_planner_wrapper/solve"
        rospy.wait_for_service(service_name)
        srv_client = rospy.ServiceProxy(service_name, Empty)

        req = EmptyRequest()
        res = srv_client.call(req)  # type: EmptyResponse
        return

    @staticmethod
    def reset_planner_client():
        service_name = "/prm_planner_wrapper/reset_planner"
        rospy.wait_for_service(service_name)
        srv_client = rospy.ServiceProxy(service_name, Empty)

        req = EmptyRequest()
        res = srv_client.call(req)  # type: EmptyResponse
        return

    @staticmethod
    def add_ovc_ct(ovc=[], interval=[], params=None, ct_type=ConstraintType.StartsAfterEnd):

        add_ovc_ct_client = rospy.ServiceProxy("/prm_planner_wrapper/add_ovc_ct", AddOvcCt)

        req = AddOvcCtRequest()
        req.ct.ovc_name = [o.name for o in ovc]
        req.ct.ct_type.val = ct_type
        req.ct.ovc_interval = interval
        if params is None:
            req.ct.params = []
        else:
            req.ct.params = params

        res = add_ovc_ct_client.call(req)  # type: AddOvcCtResponse
        return res

    @staticmethod
    def add_state_ct(group_name,state_index, values, use_location=True):
        # type: (str, int, list[int], bool) -> bool
        '''
        Restrict the value of a configuration variable identified by the group_name and the state_index to be in values.
        if use_location is True, the configuration must be in the configurations reaching to any of the locations in
        values
        :param group_name: name of the active component
        :param state_index: index to select the configuration variables. negative values are possible (e.g. -1 denotes
        the last configuration
        :param values: list of integers.
        :param use_location: determines if values are interpreted as locations or configurations
        :return: returns False, if the problem gets infeasible
        '''

        srv_set_state = rospy.ServiceProxy("/prm_planner_wrapper/set_state", SetState)

        set_state_req = SetStateRequest()
        set_state_req.group = group_name
        set_state_req.state_index = state_index
        set_state_req.values.values = values
        set_state_req.use_location = use_location
        res = srv_set_state.call(set_state_req)

        return res.success


    # TODO: factor out to motion dispatcher service clients
    @staticmethod
    def move_arm_to_node_client(input=[]):
        service_name = 'MOTION_DISPATCHER/MOVE_TO_NODE'
        rospy.wait_for_service(service_name)
        try:
            get_obj_srv = rospy.ServiceProxy(service_name, StringQuery)
            req = StringQueryRequest()  # type: StringQueryRequest
            req.input = input
            resp1 = get_obj_srv(req)  # type: StringQueryResponse
            return resp1.output, resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    # srv_add_ovc_ct = rospy.ServiceProxy("/prm_planner_wrapper/add_ovc_ct", AddOvcCt)
    # srv_set_state = rospy.ServiceProxy("/prm_planner_wrapper/set_state", SetState)
    # srv_add_obj_online = rospy.ServiceProxy("/prm_planner_wrapper/add_object_online", AddObject)

