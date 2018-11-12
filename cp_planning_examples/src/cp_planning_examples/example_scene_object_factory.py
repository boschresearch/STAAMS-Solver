#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import geometry_msgs
import visualization_msgs.msg
from roadmap_tools.scene_object_factory import SceneObjectFactory, SceneObject
from wx._aui import AuiMDIParentFrame_SetChildMenuBar


class ExampleSceneObjectFactory(SceneObjectFactory):

    def __init__(self):
        super(ExampleSceneObjectFactory, self).__init__()

    @staticmethod
    def create_scene_object(obj_type, parent_name, object_id, object_name=None, px=0.6, py=0.0, pz=0.95, ox=0.0, oy=0.0,
                            oz=0.0, ow=1.0):
        obj = SceneObject()
        obj.set_object_id(object_id)

        # if a name is given, we use it. Otherwise, we generate an individual name from the object type and the id
        if isinstance(object_name, str):
            obj.set_name(object_name)
        else:
            obj.set_name(obj_type + "_" + str(obj.object_id))
        obj.type = obj_type

        # template definitions
        if obj_type == "block":
            obj.set_has_collision(has_collision=True)
            obj.set_visualization(visualization_msgs.msg.Marker.CUBE)
            obj.set_size([0.02, 0.02, 0.05])

        if obj_type == "location":
            obj.set_has_collision(has_collision=False)
            obj.set_visualization(visualization_msgs.msg.Marker.SPHERE)
            obj.set_size([0.02, 0.02, 0.02])

        return obj

    @staticmethod
    def spawn_scene(add_object_from_template, scene_id=-1):
        if scene_id == -1:
            # empty scene
            return

        if scene_id == 0:
            add_object_from_template('block', 'workpiece', 'block_1', px=-0.1, py=0.0, pz=0.0)
            add_object_from_template('block', 'workpiece', 'block_2', px=-0.05, py=0.0, pz=0.0)
            add_object_from_template('block', 'workpiece', 'block_3', px=0.0, py=0.0, pz=0.0)
            add_object_from_template('block', 'workpiece', 'block_4', px=0.05, py=0.0, pz=0.0)
            # add_object_from_template('block', 'workpiece', 'block_5', px=0.1, py=0.0, pz=0.0)

            add_object_from_template('location', 'WAIST', 'home_left', px=-0.0, py=0.5, pz=0.05)
            add_object_from_template('location', 'WAIST', 'home_right', px=-0.0, py=-0.5, pz=0.05)

            add_object_from_template('location', 'WAIST', 'drop_left', px=0.2, py=0.5, pz=0.0)
            add_object_from_template('location', 'WAIST', 'drop_right', px=0.2, py=-0.5, pz=0.0)

            return