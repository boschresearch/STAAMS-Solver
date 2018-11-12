#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""


import math

import geometry_msgs
import tf
from roadmap_tools.robot_info import RobotInfo


class RobotInfoKawada(RobotInfo):
    def __init__(self):
        pass

    def getStateValidity(self):
        from roadmap_tools import kinematics_interface
        return kinematics_interface.StateValidity()

    def getSV(self):
        from roadmap_tools import kinematics_interface
        return kinematics_interface.StateValidity()

    def getIK(self):
        from roadmap_tools import kinematics_interface
        return kinematics_interface.InverseKinematics()

    def getFK(self):
        from roadmap_tools import kinematics_interface
        return kinematics_interface.ForwardKinematics()

    def getRoots(self, now):
        return self.getStaticRobotTransforms(now)

    def getStaticRobotTransforms(self, now):

        transforms = []

        # right arm gripper transform
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = now
        static_transformStamped.header.frame_id = "RARM_JOINT5_Link"
        static_transformStamped.child_frame_id = "RARM_Gripper"

        static_transformStamped.transform.translation.x = -0.2
        static_transformStamped.transform.translation.y = -0.00
        static_transformStamped.transform.translation.z = -0.01

        quat = tf.transformations.quaternion_from_euler(0.0, math.pi / 2, 0.0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        transforms.append(static_transformStamped)

        # left arm gripper transform
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = now
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

        transforms.append(static_transformStamped)
        return transforms

    def get_eef2tip_transform(self, group, now):
        if group == "right_arm":
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            static_transformStamped.header.stamp = now
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
            return static_transformStamped

        if group == "left_arm":
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            static_transformStamped.header.stamp = now
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
            return static_transformStamped

    def getBaseFrame(self):
        return "WAIST"

    def getGroups(self):
        return ["right_arm", "left_arm"]

    def getEndEffector(self, group):
        group2ee = {"right_arm": "RARM_JOINT5_Link", "left_arm": "LARM_JOINT5_Link"}
        return group2ee[group]

    def getCompleteGroup(self):
        return "upperbody"

    def getGroupForOVCType(self, ovc_type):
        task2group = {"pick": ["right_arm", "left_arm"]}
        return task2group[ovc_type]