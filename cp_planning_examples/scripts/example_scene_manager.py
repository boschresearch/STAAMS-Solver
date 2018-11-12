#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from roadmap_tools.scene_graph_mockup_manager import SceneGraphMockupManager
from cp_planning_examples.example_scene_object_factory import ExampleSceneObjectFactory
from geometry_msgs.msg import TransformStamped
from roadmap_tools.robot_info import RobotInfo


def get_robot_frame(world_frame, robot_base_frame):
    transform = TransformStamped()
    transform.header.frame_id = world_frame
    transform.child_frame_id = robot_base_frame
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.95
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    return transform

def get_task_frame(world_frame):
    transform = TransformStamped()
    transform.header.frame_id = world_frame
    transform.child_frame_id = "workpiece"
    transform.transform.translation.x = 0.4
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.9
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    return transform


if __name__ == "__main__":
    world_frame = 'map'
    robot_info = RobotInfo.getRobotInfo()  # type: RobotInfo

    example_scene_object_factory = ExampleSceneObjectFactory()
    example_scene_object_factory.addRobotFrame(get_robot_frame(world_frame, robot_info.getBaseFrame()))
    example_scene_object_factory.addTaskFrame(get_task_frame(world_frame))
    sggm = SceneGraphMockupManager(ref_frame=world_frame, scene_object_factory=example_scene_object_factory)