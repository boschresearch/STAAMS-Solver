#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import copy

from roadmap_tools.scene_object import SceneObject
from geometry_msgs.msg import TransformStamped
from rospy import Time


class SceneObjectFactory(object):

    def __init__(self):
        self.task_frames = []
        self.robot_frames = []

    def getStaticTransforms(self, now=None):
        if now is None:
            return self.robot_frames + self.task_frames
        else:
            assert isinstance(now, Time)
            transforms = SceneObjectFactory.set_timestamps(self.robot_frames + self.task_frames, now)
            return copy.copy(transforms)

    @staticmethod
    def set_timestamps(transforms, now):
        # type: (list[TransformStamped], Time) -> list[TransformStamped]
        assert isinstance(transforms, list)
        assert isinstance(now, Time)

        for trans in transforms:
            trans.header.stamp = now

        return transforms


    @staticmethod
    def create_scene_object(obj_type, parent_name, object_id, object_name=None, px=0.6, py=0.0, pz=0.95, ox=0.0, oy=0.0, oz=0.0, ow=1.0):
        # type: (str, str, int, float, float, float, float, float, float, float) -> SceneObject
        pass

    @staticmethod
    def spawn_scene(add_object_from_template, scene_id=-1):
        pass

    def addTaskFrame(self, task_frame):
        assert isinstance(task_frame, TransformStamped)
        if SceneObjectFactory.checkTransDuplicate(self.task_frames, task_frame):
            self.task_frames.append(task_frame)
            return True
        return False

    def addRobotFrame(self, robot_frame):
        assert isinstance(robot_frame, TransformStamped)
        if SceneObjectFactory.checkTransDuplicate(self.robot_frames, robot_frame):
            self.robot_frames.append(robot_frame)
            return True
        return False

    @staticmethod
    def checkTransDuplicate(trans_list, new_trans):
        # type: (list[TransformStamped], TransformStamped) -> bool

        for trans in trans_list:
            if trans.header.frame_id == new_trans.header.frame_id and trans.child_frame_id == new_trans.child_frame_id:
                # A transfrom for the given frames is already defined. Replace or skip.
                return False
        # No transform is defined yet. Just add the transform
        return True
