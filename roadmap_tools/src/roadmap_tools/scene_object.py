#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import geometry_msgs.msg
import visualization_msgs.msg
import rospy
import tf

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class SceneObject:

    # name = ''
    # parent_object = None
    # parent_object_name = None
    # transform_from_parent_object = geometry_msgs.msg.TransformStamped()
    # object_id = None
    #
    # has_visual = False
    # mesh_resource = 'package://nextage_movement_skills/python/meshes/tmp/whole_sweep.dae'

    def __init__(self):
        self.parent_object = None
        self.parent_object_name = None
        self.transform_from_parent_object = geometry_msgs.msg.TransformStamped()
        self.object_id = None
        self.marker_type = visualization_msgs.msg.Marker.MESH_RESOURCE
        self.mesh_resource = None
        self.name = None
        self.has_visual = False
        self.has_collision = False
        self.updated = True
        self.children = {}
        self.delete_marker = False
        self.type = 'abstract_object'
        self.color = {'r': 1.0, 'g': 0.0, 'b': 0.0}
        self.size = [1.0, 1.0, 1.0]

    def set_name(self, name):
        self.name = name

    def add_child(self, child):
        self.children[child.name] = child

    def remove_child_by_name(self, child_name):
        if child_name in self.children.keys():
            del self.children[child_name]
            return
        rospy.logwarn("Object child not deleted from " + self.name + ": not in dict")

    def set_object_id(self, idx):
        self.object_id = idx

    def set_transform_from_parent_object(self, transform):
        self.transform_from_parent_object = transform

    def set_parent_object(self, parent_object):
        self.parent_object = parent_object

    def set_parent_object_name(self, parent_object_name):
        self.parent_object_name = parent_object_name

    def set_mesh_resource(self, mesh_resource):
        self.mesh_resource = mesh_resource
        self.has_visual = True
        self.marker_type = visualization_msgs.msg.Marker.MESH_RESOURCE

    def set_visualization(self, viz_type=visualization_msgs.msg.Marker.SPHERE):
        self.marker_type = viz_type
        self.has_visual = True

    def set_size(self, size=[0.1, 0.1, 0.1]):
        self.size = size

    def set_has_collision(self, has_collision=True):
        self.has_collision = has_collision
        return self.has_collision

    def get_has_collision(self):
        return self.has_collision

    def get_visualization_marker(self, action=visualization_msgs.msg.Marker.ADD):
        # type: (int) -> visualization_msgs.Marker
        if not self.has_visual:
            rospy.logwarn("Object " + self.name + " shall not be visualized. Configure and retry!")
            return
        marker = visualization_msgs.msg.Marker()

        # marker.header.frame_id = self.parent_object_name
        marker.header.frame_id = self.name
        marker.header.stamp = rospy.Time.now()
        marker.ns = "scene_objects"
        marker.id = self.object_id
        marker.type = self.marker_type
        marker.action = action

        if self.delete_marker:
            marker.action = visualization_msgs.msg.Marker.DELETE

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.scale.x = self.size[0]
        marker.scale.y = self.size[1]
        marker.scale.z = self.size[2]
        marker.color.a = 0.9
        marker.color.r = self.color['r']
        marker.color.g = self.color['g']
        marker.color.b = self.color['b']

        if self.marker_type == visualization_msgs.msg.Marker.MESH_RESOURCE:
            marker.mesh_resource = self.mesh_resource
            assert marker.mesh_resource is not None

        return marker

    def set_color(self, r=1.0, g=1.0, b=1.0):
        self.color['r'] = r
        self.color['g'] = g
        self.color['b'] = b

    def reparent(self, new_parent_transform, new_parent):
        del self.parent_object.children[self.name]
        old_parent = self.parent_object
        self.parent_object = new_parent
        self.parent_object.add_child(self)
        self.parent_object_name = new_parent.name
        self.transform_from_parent_object = new_parent_transform

        rospy.loginfo("Object " + self.name + " reparented from " + old_parent.name + " to " + self.parent_object.name)

    # def get_grasp_poses(self, end_effector):



