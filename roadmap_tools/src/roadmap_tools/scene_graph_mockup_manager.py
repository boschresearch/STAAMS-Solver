#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from roadmap_tools.scene_object import SceneObject
import visualization_msgs.msg
import rospy

from visualization_msgs.msg import Marker, MarkerArray

import copy
import math

import tf

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

import tf2_geometry_msgs

from roadmap_planning_common_msgs.srv import StringQuery, StringQueryResponse, AddObject, AddObjectResponse, AddObjectRequest

import moveit_commander

import StringIO
import cPickle as pickle

from roadmap_tools.robot_info import RobotInfo

from roadmap_tools.scene_object_factory import SceneObjectFactory


class SceneGraphMockupManager:

    def __init__(self, ref_frame='map', robot_name='kawada', scene_object_factory=None):
        # self.scene_objects = []
        rospy.init_node("SceneGraphMockupManager")

        # TODO: make the roots configurable

        # we save object names here, that should not be deleted
        self.keep_frames = set()

        # assert isinstance(scene_object_factory, SceneObjectFactory)
        self.scene_object_factory = scene_object_factory  #  type: SceneObjectFactory
        self.robot_info = RobotInfo.getRobotInfo()  # type: RobotInfo

        self.root = ['map']
        self.scene_objects_dict = {}

        self.static_transforms = []
        # self.static_transforms += self.robot_info.getStaticRobotTransforms(rospy.Time.now())
        self.static_transforms += self.scene_object_factory.getStaticTransforms(rospy.Time.now())
        self.add_transforms(self.static_transforms)
        # self.add_transforms(self.scene_object_factory.getStaticTransforms(rospy.Time.now()))

        topic = 'visualization_marker_array'
        self.marker_publisher = rospy.Publisher(topic, visualization_msgs.msg.MarkerArray, queue_size=100)
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.timer = rospy.Timer(rospy.Duration(0.5), self.handle_publish_transforms_timer)
        rospy.on_shutdown(self.shutdown_hook)

        rospy.sleep(1.0)

        # TODO: factor static transforms out
        # self.gripper_static_transform(robot_name=robot_name)
        # self.init_robot_transform(robot_name=robot_name)
        self.init_tree_roots(ref_frame=ref_frame, robot_name=robot_name)

        # Open service for object query by type
        self.object_name_by_type = rospy.Service('SceneGraphMockupManager/getObjectNamesByType', StringQuery,
                                                 self.handle_get_object_names_by_type)
        self.object_types_in_scene = rospy.Service('SceneGraphMockupManager/getObjectTypesInScene', StringQuery,
                                                   self.handle_get_object_types)
        self.object_pose_by_name = rospy.Service('SceneGraphMockupManager/getObjectPoseByName', StringQuery,
                                                 self.handle_get_object_poses_by_names)
        self.object_del_by_name = rospy.Service('SceneGraphMockupManager/delObjectsByName', StringQuery,
                                                self.handle_del_objects_by_name)
        self.objects_by_type = rospy.Service('SceneGraphMockupManager/getObjectsByType', StringQuery,
                                             self.handle_get_objects_by_type)
        self.set_object_color_by_name = rospy.Service('SceneGraphMockupManager/setObjectColorByName', StringQuery,
                                                      self.handle_set_object_color_by_name)
        self.reload_scene = rospy.Service('SceneGraphMockupManager/reloadSceneById', StringQuery,
                                          self.handle_reload_scene_by_id)
        self.add_object_srv = rospy.Service('SceneGraphMockupManager/addObject', AddObject,
                                            self.handle_add_object)
        self.get_object_types_by_name = rospy.Service('SceneGraphMockupManager/getObjectTypesByName', StringQuery,
                                                      self.handle_get_object_types_by_name)
        rospy.spin()

    def add_transforms(self, transforms):
        if isinstance(transforms, list):
            for trans in transforms:  # type: TransformStamped
                o = SceneObject()
                o.set_name(trans.child_frame_id)
                o.set_transform_from_parent_object(trans)
                o.set_parent_object(trans.header.frame_id)
                o.set_parent_object_name(trans.header.frame_id)
                self.add_object(o)

    def handle_get_object_types_by_name(self, req):
        res = StringQueryResponse()
        obj_types = []
        for obj_name in req.input:
            obj = self.scene_objects_dict[obj_name]  # type: SceneObject
            obj_types.append(obj.type)
        res.output = obj_types
        res.success = True
        return res

    def handle_get_object_types(self, req):
        res = StringQueryResponse()
        obj_types = set()
        for obj in self.scene_objects_dict.values(): # type: SceneObject
            # obj = self.scene_objects_dict[obj_name]
            obj_types.update(obj.type)
        res.output = list(obj_types)
        res.success = True
        return res

    def handle_get_objects_by_type(self, req):
        res = StringQueryResponse()
        for obj_type in req.input:
            # s = StringIO.StringIO()
            # self.get_eef_pose_for_grasp(obj_name).serialize(s)

            objects = self.get_objects(obj_type)
            for key, obj in objects.items():
                obj_str = pickle.dumps(obj)
                # obj.serialize(s)
                res.output.append(obj_str)

        return res

    def handle_del_objects_by_name(self, req):
        '''

        :param req: req.input is a list of object names
        :return: list of deleted objects
        '''
        res = StringQueryResponse()

        suc = self.remove_objects(req.input)
        return res


        # for obj_name in req.input:
        #     if self.remove_object(obj_name):
        #         res.output.append(obj_name)
        # return res

    def handle_add_object(self, req):
        # type: (AddObjectRequest) -> AddObjectResponse
        '''
        Adding an object to the scene graph
        :param req:
        :return:
        '''
        res = AddObjectResponse()

        if self.get_scene_object_by_name(req.pose.header.frame_id) is None:
            res.success = False
            rospy.logerr("No object with name {} in scene graph.".format(req.pose.header.frame_id))
            return res

        # TODO: name is missing
        self.add_object_from_template(req.object_type,
                                      req.pose.header.frame_id,
                                      req.pose.pose.position.x,
                                      req.pose.pose.position.y,
                                      req.pose.pose.position.z,
                                      req.pose.pose.orientation.x,
                                      req.pose.pose.orientation.y,
                                      req.pose.pose.orientation.z,
                                      req.pose.pose.orientation.w)

        return res

    def handle_reload_scene_by_id(self, req):
        res = StringQueryResponse()

        scene_id = int(req.input[0])

        obj_names = copy.copy(self.scene_objects_dict.keys())

        for i in range(self.scene_objects_dict.keys().__len__()):

            del_objs = self.scene_objects_dict.keys()
            for k in self.keep_frames:
                del_objs.remove(k)
            self.remove_objects(del_objs)

            # for obj_name in self.scene_objects_dict.keys():
            #     if obj_name not in self.keep_frames:
            #         self.remove_object(obj_name)

        # for obj_name in obj_names:
        #     if "_r" not in obj_name:
        #         if self.scene_objects_dict[obj_name].has_visual:
        #             self.remove_object(obj_name)


        rospy.sleep(1.0)
        self.delete_planning_scene()

        self.add_transforms(self.static_transforms)

        self.scene_object_factory.spawn_scene(self.add_object_from_template, scene_id=scene_id)
        # self.define_test_scene_graph(scene_id)

        return res

    def handle_set_object_color_by_name(self, req):
        '''

        :param req: req.input is a four-tuple: object name, red, green, blue
        :return: list of deleted objects
        '''
        res = StringQueryResponse()

        obj_name = req.input[0]
        r = float(req.input[1])
        g = float(req.input[2])
        b = float(req.input[3])

        # TODO: add test
        if obj_name in self.scene_objects_dict.keys():
            self.scene_objects_dict[obj_name].set_color(r=r, g=g, b=b)
        return res

    def handle_get_object_poses_by_names(self, req):
        res = StringQueryResponse()
        for obj_name in req.input:
            s = StringIO.StringIO()
            self.get_eef_pose_for_grasp(obj_name).serialize(s)

            s2 = ""
            for f in s.buflist:
                s2 += f
            res.output.append(s2)

        return res

    def handle_get_object_names_by_type(self, req):
        res = StringQueryResponse()
        objs = self.get_objects(req.input)
        res.output = objs.keys()
        res.success = True
        return res

    def add_object(self, obj):
        if obj.parent_object in self.root:
            # self.scene_objects.append(obj)
            self.scene_objects_dict[obj.name] = obj
            rospy.loginfo("Object " + obj.name + "  added to Scene Graph - parent is root: " + obj.parent_object_name)
            return
        if obj.parent_object.name in self.scene_objects_dict.keys():
            # self.scene_objects.append(obj)
            self.scene_objects_dict[obj.name] = obj
            self.scene_objects_dict[obj.parent_object_name].add_child(obj)
            rospy.loginfo("Object " + obj.name + " added to Scene Graph - parent present: " + obj.parent_object_name)
            return
        else:
            rospy.logwarn("Object " + obj.name + "  not added to Scene Graph - parent not present: " + obj.parent_object_name)
            return

    def remove_objects(self, del_names):
        del_obj = []
        for name in del_names:
            if name in self.scene_objects_dict.keys() and not self.scene_objects_dict[name].children:
                if isinstance(self.scene_objects_dict[name].parent_object, SceneObject):
                    self.scene_objects_dict[name].parent_object.remove_child_by_name(name)
                    self.scene_objects_dict[name].delete_marker = True
                    self.scene_objects_dict[name].updated = True
                del_obj.append(name)

        self.vis_scene_graph()
        self.handle_publish_transforms_timer('bla')

        for name in del_obj:
            del self.scene_objects_dict[name]

        return True

    def remove_object(self, del_name):
        if del_name in self.scene_objects_dict.keys() and not self.scene_objects_dict[del_name].children:
            self.scene_objects_dict[del_name].parent_object.remove_child_by_name(del_name)
            self.scene_objects_dict[del_name].delete_marker = True
            self.scene_objects_dict[del_name].updated = True
            self.delete_planning_scene_object([del_name])
            self.vis_scene_graph()
            self.handle_publish_transforms_timer('bla')

            # rospy.loginfo("Object " + self.scene_objects_dict[del_name].name + " removed from scene graph")
            rospy.loginfo("Object " + del_name + " removed from scene graph")
            del self.scene_objects_dict[del_name]

            return True
        else:
            if del_name in self.scene_objects_dict.keys():
                rospy.loginfo("Object " + self.scene_objects_dict[del_name].name + " NOT removed from scene graph. " +
                              "Object present and has children: {}".format(self.scene_objects_dict[del_name].children))

            return False

    def handle_publish_transforms_timer(self, event):
        if self.br is None:
            self.br = tf2_ros.TransformBroadcaster()
            rospy.sleep(1.0)

        for trans in self.static_transforms:
            trans.header.stamp = rospy.Time.now()
            self.br.sendTransform(trans)

        for obj in self.scene_objects_dict.values():
            obj.transform_from_parent_object.header.stamp = rospy.Time.now()
            self.br.sendTransform(obj.transform_from_parent_object)
            # obj.updated = True
            # TODO: updated should only be set to True, if the object needs new visualization

        self.publish_planning_scene()
        self.vis_scene_graph()

        # for obj in self.scene_objects_dict.values():
        #     if obj.delete_marker:
        #         del self.scene_objects_dict[obj.name]

    def publish_planning_scene(self):
        # robot = moveit_commander.RobotCommander()
        if self.scene is None:
            self.scene = moveit_commander.PlanningSceneInterface()
            rospy.sleep(1.0)

        # self.scene = moveit_commander.PlanningSceneInterface()

        for obj in self.scene_objects_dict.values():
            if obj.get_has_collision() and obj.updated and not obj.delete_marker:
                # TODO: remove ugly hack with visualization marker for pose
                marker = obj.get_visualization_marker()  # type:Marker
                # marker.pose.position.z += 0.0251
                # TODO: This just fits for the block object. We should read the marker from the SceneObject
                if marker.type == Marker.CUBE:
                    self.scene.add_box(obj.name, marker, [marker.scale.x, marker.scale.y, marker.scale.z])

                # self.scene.add_box(obj.name, obj.get_visualization_marker(), [0.02, 0.02, 0.05])
            elif obj.delete_marker and obj.get_has_collision():
                self.scene.remove_world_object(obj.name)


    def delete_planning_scene(self):
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object()

        # for obj_name in scene.get_known_object_names():
        #     scene.remove_world_object(obj_name)

    def delete_planning_scene_object(self, obj_names=[]):
        if not self.scene:
            self.scene = moveit_commander.PlanningSceneInterface()
            rospy.sleep(1.0)

        scene = self.scene
        world_objects = scene.get_known_object_names()

        # if 'part_placeholder_14' in obj_names:
        #     print("pl 14")

        for obj_name in obj_names:
            if obj_name in world_objects:
                while obj_name in scene.get_known_object_names():
                    scene.remove_world_object(obj_name)


        # for obj_name in scene.get_known_object_names():
        #     if obj_name in obj_names:
        #         scene.remove_world_object(obj_name)
        #         # TODO: remove sleep after debugging
        #         rospy.sleep(0.1)

    def vis_scene_graph(self):

        if self.marker_publisher is None:
            topic = 'visualization_marker_array'
            self.marker_publisher = rospy.Publisher(topic, visualization_msgs.msg.MarkerArray, queue_size=100)
            rospy.sleep(1.0)

        marker_array = visualization_msgs.msg.MarkerArray()

        obj2delete = []

        for obj in self.scene_objects_dict.values():
            # obj.updated = True
            if obj.has_visual is True and obj.updated:
                obj.updated = False
                marker = obj.get_visualization_marker()
                if obj.delete_marker:
                    print("delete status : {}".format(marker.action))
                    marker.action = visualization_msgs.msg.Marker.DELETE
                    print("delete status after setting it: {}".format(marker.action))
                marker_array.markers.append(marker)
                # if obj.delete_marker:
                #     marker = marker_array.markers[-1]  # type: visualization_msgs.msg.Marker
                #     print(marker.action)
                #     marker.action = visualization_msgs.msg.Marker.DELETE
                #     obj2delete.append(obj.name)

        if marker_array.markers.__len__() > 0:
            self.marker_publisher.publish(marker_array)

        # for obj_name in obj2delete:
        #     marker_array = visualization_msgs.msg.MarkerArray()
        #     marker = self.scene_objects_dict[obj.name].get_visualization_marker()
        #     marker.action = 2
        #     marker_array.markers.append(marker)
        #     self.marker_publisher.publish(marker_array)
        #     # del self.scene_objects_dict[obj_name]

    # TODO: delete
    def init_robot_transform(self, robot_name='kawada'):
        # br = tf2_ros.TransformBroadcaster()

        if robot_name == 'kawada':
            robot_trans = SceneObject()
            robot_trans.set_parent_object_name('map_r')
            # robot_trans.set_name('WAIST')

            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map_r"
            t.child_frame_id = "WAIST"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.95
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
        else:
            return
        self.static_transform_broadcaster.sendTransform(t)

        # TODO: send as static transform

        robot_trans.set_transform_from_parent_object(t)

        robot_trans.set_name('WAIST')
        self.scene_objects_dict[robot_trans.name] = robot_trans

        self.keep_frames.add(robot_trans.name)

    def get_scene_object_by_name(self, name, default=None):
        if name in self.scene_objects_dict.keys():
            return self.scene_objects_dict.get(name)
        else:
            rospy.logwarn('No scene object with name ' + name + ' present')
            return default

    def get_object_id(self):
        obj_ids = {}
        for obj in self.scene_objects_dict.values():
            obj_ids[obj.object_id] = obj

        for new_id in range(0, self.scene_objects_dict.keys().__len__() + 1):
            if new_id not in obj_ids.keys():
                return new_id
        assert False

    def calc_invariant_transform_for_parent(self, parent_name, object_name):
        if parent_name in self.scene_objects_dict.keys() and object_name in self.scene_objects_dict.keys():
            new_parent = self.scene_objects_dict[parent_name]
            obj = self.scene_objects_dict[object_name]

            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            try:
                trans = self.tfBuffer.lookup_transform(parent_name, object_name, rospy.Time(0), rospy.Duration(1.0))
                # pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
                return trans
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e



    # def add_object_template(self, type="part", mesh_resource="package://nextage_bosch_workspace_simulation/models/brush/meshes/brush_visual.dae"):
    #     object_template_values = {}
    #     object_template_values["mesh_resource"] = mesh_resource
    #
    #     self.object_templates[type] = object_template_values
    #
    def add_object_from_template(self, obj_type='brush', parent_name='map_r', object_name=None, px=0.6, py=0.0, pz=0.95, ox=0.0, oy=0.0, oz=0.0, ow=1.0):

        # test if parent is present
        if parent_name not in self.scene_objects_dict.keys():
            rospy.logerr('No scene object with name ' + parent_name + ' present. Needed as parent.')
            return False

        # create object id
        obj_id = self.get_object_id()

        # test if object name is unique
        if object_name in self.scene_objects_dict.keys():
            object_name = None
            rospy.logwarn("Given object name {} is not unique. I will use {}_{} as name.".format(object_name, obj_type, obj_id))

        # create object
        obj = self.scene_object_factory.create_scene_object(obj_type=obj_type, parent_name=parent_name, object_id=obj_id, object_name=object_name, px=px, py=py, pz=pz, ox=ox, oy=oy, oz=oz, ow=ow)  # type: SceneObject

        # store reference to parent
        obj.set_parent_object(self.get_scene_object_by_name(parent_name))
        obj.set_parent_object_name(parent_name)

        # create transform from parent to object
        trans = geometry_msgs.msg.TransformStamped()
        trans.header.frame_id = parent_name
        trans.child_frame_id = obj.name
        trans.transform.translation.x = px
        trans.transform.translation.y = py
        trans.transform.translation.z = pz
        trans.transform.rotation.x = ox
        trans.transform.rotation.y = oy
        trans.transform.rotation.z = oz
        trans.transform.rotation.w = ow

        obj.set_transform_from_parent_object(trans)

        # add object to scene graph
        self.add_object(obj)

        # trigger visualization
        self.handle_publish_transforms_timer('add_templated_object')

        return True

    def get_objects(self, types):
        objs = copy.copy(self.scene_objects_dict)

        for obj_key in objs.keys():
            if objs[obj_key].type not in types:
                del objs[obj_key]

        return objs

    def spawn_scene(self, scene_id=-1):
        self.scene_object_factory.spawn_scene(self.add_object_from_template, scene_id=scene_id)

    def define_test_scene_graph(self, scene_id=0):

        if scene_id == -1:
            # empty scene
            return

        if scene_id == 0:
            # add home pose
            obj_type = "home_pose_left"
            px = 0.0
            py = 0.5
            pz = 1.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

            # add home pose
            obj_type = "home_pose_right"
            px = 0.0
            py = -0.5
            pz = 1.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

            self.add_object_from_template("torpedo", px=0.2, py=-0.45, pz=0.8)
            self.add_object_from_template("torpedo", px=0.2, py=0.45, pz=0.8)

            for i in range(5):
                self.add_object_from_template(obj_type="brush", px=0.45, py=-0.2 + i * 0.1, pz=0.9)
                self.add_object_from_template(obj_type="plate", px=0.3, py=-0.3 - 0.05 + i * 0.15, pz=0.8)

            return

        if scene_id == 1:
            import numpy as np
            x_nr = 4
            y_nr = 12
            x_min = 0.3
            x_max = 0.5
            y_min = -0.3
            y_max = 0.3
            z = 0.9
            x_dist = ((x_max - x_min - 0.01) / (x_nr - 1))
            y_dist = ((y_max - y_min - 0.01) / (y_nr - 1))
            dest_mat = np.random.randint(2, size=(x_nr, y_nr))

            x = np.arange(x_min, x_max, x_dist)
            y = np.arange(y_min, y_max, y_dist)

            import itertools
            it = itertools.product(x, y)
            for i in enumerate(it):
                print i
                xi = i[0] / y_nr
                yi = i[0] % y_nr
                px = i[1][0]
                py = i[1][1]
                pz = z
                if dest_mat[xi][yi] == 0:
                    obj_type = "brush"
                else:
                    obj_type = "torpedo"
                self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

        if scene_id == 2:
            import numpy as np
            x_nr = 4
            y_nr = 12
            x_min = 0.3
            x_max = 0.5
            y_min = -0.3
            y_max = 0.3
            if rospy.get_param("/SolverSetup/ROBOT_NAME", "kawada") == "kuka":
                z = 0.0
            else:
                z = 0.9
            x_dist = ((x_max - x_min - 0.01) / (x_nr - 1))
            y_dist = ((y_max - y_min - 0.01) / (y_nr - 1))
            dest_mat = np.random.randint(2, size=(x_nr, y_nr))

            x = np.arange(x_min, x_max, x_dist)
            y = np.arange(y_min, y_max, y_dist)

            import itertools
            it = itertools.product(x, y)
            for i in enumerate(it):
                print i
                xi = i[0] / y_nr
                yi = i[0] % y_nr
                px = i[1][0]
                py = i[1][1]
                pz = z
                if dest_mat[xi][yi] == 0:
                    obj_type = "brush"
                else:
                    obj_type = "torpedo"

                obj_type = "part_placeholder"
                self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

            # add the box placeholders
            obj_type = "store_placeholder_right"
            px = 0.4
            py = -0.4
            if rospy.get_param("/SolverSetup/ROBOT_NAME", "kawada") == "kuka":
                pz = 0.0
            else:
                pz = 0.9
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

            obj_type = "store_placeholder_left"
            px = 0.4
            py = 0.4
            if rospy.get_param("/SolverSetup/ROBOT_NAME", "kawada") == "kuka":
                pz = 0.0
            else:
                pz = 0.9
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

            # add home pose
            obj_type = "home_pose_left"
            if rospy.get_param("/SolverSetup/ROBOT_NAME", "kawada") == "kuka":
                px = 0.3
                py = 0.5
                pz = 0.2
            else:
                px = 0.0
                py = 0.5
                pz = 1.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

            # add home pose
            obj_type = "home_pose_right"
            if rospy.get_param("/SolverSetup/ROBOT_NAME", "kawada") == "kuka":
                px = 0.5
                py = -0.5
                pz = 0.2
            else:
                px = 0.0
                py = -0.5
                pz = 1.0

            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

        if scene_id == 3:
            # glue point scene

            # add workpiece_frame
            obj_type = "workpiece"
            px = 0.0
            py = 0.0
            pz = 0.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz)

            wp = self.get_objects(["workpiece"])

            # add glue point
            obj_type = "gluepoint"
            px = 0.1
            py = 0.0
            pz = 0.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz, parent_name=wp.keys()[-1])

            obj_type = "gluepoint"
            px = 0.15
            py = 0.0
            pz = 0.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz, parent_name=wp.keys()[-1])

            obj_type = "gluepoint"
            px = 0.1
            py = 0.1
            pz = 0.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz, parent_name=wp.keys()[-1])

            obj_type = "gluepoint"
            px = 0.15
            py = 0.1
            pz = 0.0
            self.add_object_from_template(obj_type=obj_type, px=px, py=py, pz=pz, parent_name=wp.keys()[-1])

    def init_tree_roots(self, ref_frame='map', robot_name='kawada'):
        obj_2 = SceneObject()
        obj_2.set_parent_object('map')
        obj_2.set_parent_object_name('map')
        obj_2.set_name('map_r')

        self.keep_frames.add(obj_2.name)

        trans = geometry_msgs.msg.TransformStamped()
        trans.header.frame_id = ref_frame
        trans.child_frame_id = obj_2.name
        trans.header.stamp = rospy.Time.now()
        trans.transform.translation.x = 0.0
        trans.transform.translation.y = 0.0
        trans.transform.translation.z = 0.0
        trans.transform.rotation.x = 0
        trans.transform.rotation.y = 0
        trans.transform.rotation.z = 0
        trans.transform.rotation.w = 1

        # TODO: set this as static transform
        broadcaster = self.static_transform_broadcaster
        broadcaster.sendTransform(trans)

        obj_2.set_transform_from_parent_object(trans)

        obj_2.has_visual = False

        obj_2.set_object_id(self.get_object_id())
        self.add_object(obj_2)
        self.keep_frames.add(obj_2.name)

        root_trans = self.robot_info.getStaticRobotTransforms(rospy.Time.now())
        for rt in root_trans:  # type: TransformStamped
            obj_2 = SceneObject()
            obj_2.set_parent_object(rt.header.frame_id)
            obj_2.set_parent_object_name(rt.header.frame_id)
            obj_2.set_name(rt.child_frame_id)

            obj_2.set_transform_from_parent_object(rt)

            obj_2.has_visual = False

            obj_2.set_object_id(self.get_object_id())

            if obj_2.parent_object not in self.root:
                self.root.append(obj_2.parent_object)
            self.add_object(obj_2)
            self.keep_frames.add(obj_2.name)

    def get_object_pose(self, object_name="map_r", frame="map_r"):
        if object_name not in self.scene_objects_dict.keys() or frame not in self.scene_objects_dict.keys():
            rospy.logerr("Can't get Pose for object " + object_name + ".\n Object or frame (" + frame + ") unknown.")
            return
        pose = geometry_msgs.msg.PoseStamped()
        try:
            trans = self.tfBuffer.lookup_transform(frame, object_name, rospy.Time(0), rospy.Duration(1.0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return pose_transformed

    def get_eef_pose_for_grasp(self, obj_frame):
        robot_name = rospy.get_param("SolverSetup" + '/ROBOT_NAME')

        pose = geometry_msgs.msg.PoseStamped()
        try:

            if robot_name == "kuka":
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.position.z = 0.08
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 1.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 0.0

            if robot_name == "kawada":
                # static_transformStamped.transform.translation.x = -0.2
                # static_transformStamped.transform.translation.y = -0.00
                # static_transformStamped.transform.translation.z = -0.01
                #
                # quat = tf.transformations.quaternion_from_euler(0.0, math.pi / 2, 0.0)
                # static_transformStamped.transform.rotation.x = quat[0]
                # static_transformStamped.transform.rotation.y = quat[1]
                # static_transformStamped.transform.rotation.z = quat[2]
                # static_transformStamped.transform.rotation.w = quat[3]


                pose.pose.position.x = -0.01
                pose.pose.position.y = 0.0
                pose.pose.position.z = 0.2 + 0.05
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = -0.707107
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 0.707107

            # planning_frame = 'map_r'
            planning_frame = self.robot_info.getBaseFrame()

            # TODO: query for transformations from object frame to end-effector in grasp/pregrasp pose
            rospy.logdebug("Using fixed pre-grasp pose. Real grasp pose service not implemented!")

            # trans = self.tfBuffer.lookup_transform('RARM_Gripper', 'RARM_JOINT5_Link', rospy.Time(0), rospy.Duration(1.0))
            # we get a pre grasp pose
            # trans.transform.translation.z += 0.08
            pose.header.frame_id = obj_frame
            # pose.pose.position = trans.transform.translation
            # pose.pose.orientation = trans.transform.rotation

            trans2 = self.tfBuffer.lookup_transform(planning_frame, obj_frame,  rospy.Time(0), rospy.Duration(1.0))

            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans2)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return pose_transformed

    def shutdown_hook(self):
        self.timer.shutdown()
        rospy.sleep(1.0)
        for i in range(self.scene_objects_dict.keys().__len__()):
            self.remove_objects(self.scene_objects_dict.keys())
            for obj_name in self.scene_objects_dict.keys():
                if self.scene_objects_dict[obj_name].has_visual:
                    self.remove_object(obj_name)
        self.delete_planning_scene()
        rospy.sleep(1.0)
        rospy.loginfo("Shutdown complete")

    def gripper_static_transform(self, robot_name='kawada'):
        #broadcaster = self.static_transform_broadcaster
        broadcaster = tf2_ros.TransformBroadcaster()

        rospy.sleep(1.0)

        static_transformStamped = geometry_msgs.msg.TransformStamped()

        if robot_name == 'kawada':
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "RARM_JOINT5_Link"
            static_transformStamped.child_frame_id = "RARM_Gripper"

            static_transformStamped.transform.translation.x = -0.2
            static_transformStamped.transform.translation.y = -0.00
            static_transformStamped.transform.translation.z = -0.01

            quat = tf.transformations.quaternion_from_euler(0.0, math.pi/2, 0.0)
            static_transformStamped.transform.rotation.x = quat[0]
            static_transformStamped.transform.rotation.y = quat[1]
            static_transformStamped.transform.rotation.z = quat[2]
            static_transformStamped.transform.rotation.w = quat[3]

            broadcaster.sendTransform(static_transformStamped)

            static_transformStamped = geometry_msgs.msg.TransformStamped()

            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "LARM_JOINT5_Link"
            static_transformStamped.child_frame_id = "LARM_Gripper"

            static_transformStamped.transform.translation.x = -0.2
            static_transformStamped.transform.translation.y = 0.00
            static_transformStamped.transform.translation.z = -0.01

            quat = tf.transformations.quaternion_from_euler(0.0, math.pi / 2, 0.0)
            static_transformStamped.transform.rotation.x = quat[0]
            static_transformStamped.transform.rotation.y = quat[1]
            static_transformStamped.transform.rotation.z = quat[2]
            static_transformStamped.transform.rotation.w = quat[3]
        else:
            return

        broadcaster.sendTransform(static_transformStamped)

    def init(self):
        # rospy.init_node('SceneGraphMockupManager')
        # sgmm = SceneGraphMockupManager()
        # rospy.on_shutdown(self.shutdown_hook)


        # rospy.Timer(rospy.Duration(0.05), self.handle_publish_transforms_timer)

        self.define_test_scene_graph()

        # rospy.spin()

    # def handle_std_srv(self, req):



if __name__ == '__main__':
    # TODO: create a standalone testcase here
    # rospy.init_node('SceneGraphMockupManager')
    robot_name = rospy.get_param('SolverSetup/ROBOT_NAME', None)
    if robot_name is None:
        rospy.logerr("Robot name not specified.")
    else:
        from roadmap_tools.sorting_scene_object_factory import SortingSceneObjectFactory
        sgmm = SceneGraphMockupManager(ref_frame='world', robot_name=robot_name, scene_object_factory=SortingSceneObjectFactory)



