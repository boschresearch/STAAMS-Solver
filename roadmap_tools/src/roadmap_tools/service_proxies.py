#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from geometry_msgs.msg import PoseStamped

import rospy

from nextage_movement_skills.srv import StringQueryResponse, StringQuery, StringQueryRequest, AddPose, AddPoseRequest, \
    AddObject, AddObjectRequest
from std_srvs.srv import SetBoolRequest, SetBool

import cPickle as pickle


class ServiceProxies:
    @staticmethod
    def get_object_names_by_type_client(input=[]):
        rospy.wait_for_service('SceneGraphMockupManager/getObjectNamesByType')
        try:
            get_obj_srv = rospy.ServiceProxy('SceneGraphMockupManager/getObjectNamesByType', StringQuery)
            req = StringQueryRequest()  # type: StringQueryRequest
            req.input = input
            resp1 = get_obj_srv(req)  # type: StringQueryResponse
            return resp1.output, resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def get_object_types_in_scene():
        service_name = 'SceneGraphMockupManager/getObjectTypesInScene'
        rospy.wait_for_service(service_name)
        input_ = []
        try:
            reload_scene_srv = rospy.ServiceProxy(service_name, StringQuery)
            req = StringQueryRequest()  # type: StringQueryRequest
            req.input = input_
            resp1 = reload_scene_srv(req)  # type: StringQueryResponse
            return resp1.output, resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def get_object_type_by_name(input=[]):
        service_name = 'SceneGraphMockupManager/getObjectTypesByName'
        rospy.wait_for_service(service_name)
        try:
            get_obj_srv = rospy.ServiceProxy(service_name, StringQuery)
            req = StringQueryRequest()  # type: StringQueryRequest
            req.input = input
            resp1 = get_obj_srv(req)  # type: StringQueryResponse
            return resp1.output, resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def reload_scene_graph_client(input=[]):
        service_name = 'SceneGraphMockupManager/reloadSceneById'
        rospy.wait_for_service(service_name)
        try:
            reload_scene_srv = rospy.ServiceProxy(service_name, StringQuery)
            req = StringQueryRequest()  # type: StringQueryRequest
            req.input = input
            resp1 = reload_scene_srv(req)  # type: StringQueryResponse
            return resp1.output, resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    @staticmethod
    def get_object_poses_by_name_client(input=[]):
        rospy.wait_for_service('SceneGraphMockupManager/getObjectPoseByName')
        try:
            get_obj_srv = rospy.ServiceProxy('SceneGraphMockupManager/getObjectPoseByName', StringQuery)
            req = StringQueryRequest()
            req.input = input
            resp1 = get_obj_srv.call(req)
            poses = []
            for s in resp1.output:
                pose = PoseStamped().deserialize(s)
                poses.append(pose)

            return poses, resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def del_objects_by_name_client(input=[]):
        service_name = 'SceneGraphMockupManager/delObjectsByName'
        rospy.wait_for_service(service_name)
        try:
            get_obj_srv = rospy.ServiceProxy(service_name, StringQuery)
            req = StringQueryRequest()
            req.input = input
            resp1 = get_obj_srv(req)
            deleted_objs = []
            for s in resp1.output:
                name = s
                deleted_objs.append(name)

            return deleted_objs
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def set_object_color_by_name_client(input=[]):
        service_name = 'SceneGraphMockupManager/setObjectColorByName'
        rospy.wait_for_service(service_name)
        try:
            get_obj_srv = rospy.ServiceProxy(service_name, StringQuery)
            req = StringQueryRequest()
            req.input = input
            resp1 = get_obj_srv(req)
            ans = []
            for s in resp1.output:
                name = s
                ans.append(name)

            return ans
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def get_objects_by_type_client(input=[]):
        rospy.wait_for_service('SceneGraphMockupManager/getObjectsByType')
        try:
            get_obj_srv = rospy.ServiceProxy('SceneGraphMockupManager/getObjectsByType', StringQuery)
            req = StringQueryRequest()
            req.input = input
            resp1 = get_obj_srv(req)
            objects = []
            for s in resp1.output:
                object_from_str = pickle.loads(s)
                objects.append(object_from_str)

            return objects, resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == '__main__':
    pass
