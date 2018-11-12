#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import copy

from roadmap_tools.prm import RoadMap


class SceneObjectSearchMapping:
    def __init__(self, roadmaps=[], old_sosm=None):
        # self._groups = []
        self._rm = {}  # type: dict[str, RoadMap]
        for rm in roadmaps:
            self._rm[rm.get_group_name()] = rm

        self._object_names = set()

        self._rm_node = {}
        for gn in self._rm.keys():
            self._rm_node[gn] = {}
        self._name_int_alias = {}
        self._object_type = {}

        self.lookup_object_names_from_roadmaps()
        self.lookup_rm_nodes_for_objects()
        self.lookup_object_types()
        if old_sosm is None:
            self.create_int_alias()
        else:
            assert isinstance(old_sosm, SceneObjectSearchMapping)
            self.create_int_alias(old_sosm.get_name_int_alias())
        print "still here"

    def get_groups(self):
        available_groups = {}
        for gn in enumerate(self._rm.keys()):
            available_groups[gn[-1]] = gn[0]
        return available_groups

    def get_name_for_alias(self, alias):
        if alias in self._name_int_alias.values():
            index = self._name_int_alias.values().index(alias)
            name = self._name_int_alias.keys()[index]
            return name
        return None

    def get_alias_to_poses(self):
        objs = self.get_available_objects()

        av_objs = {}
        for gn in self._rm.keys():
            av_objs[gn] = {}

        for on, alias in objs.items():
            for gn in self._rm.keys():
                try:
                    av_objs[gn][alias] = self._rm_node[gn][on]
                except KeyError:
                    pass
        return av_objs

    def get_alias_to_poses_for_type(self, obj_type):
        objs = self.get_available_objects_of_type(obj_type)

        av_objs = {}
        for gn in self._rm.keys():
            av_objs[gn] = {}

        for on, alias in objs.items():
            for gn in self._rm.keys():
                try:
                    av_objs[gn][alias] = self._rm_node[gn][on]
                except KeyError:
                    pass
        return av_objs

    def get_alias_to_poses_for_types(self, obj_types=[]):
        objs = self.get_available_objects_of_types(obj_types)

        av_objs = {}
        for gn in self._rm.keys():
            av_objs[gn] = {}

        for on, alias in objs.items():
            for gn in self._rm.keys():
                try:
                    av_objs[gn][alias] = self._rm_node[gn][on]
                except KeyError:
                    pass
        return av_objs

    def get_available_objects(self):
        objs = {}
        for obj, o_type in self._object_type.items():
            objs[obj] = self._name_int_alias[obj]

        return objs

    def get_available_objects_of_type(self, obj_type):
        objs = {}
        for obj, o_type in self._object_type.items():
            if obj_type == o_type:
                objs[obj] = self._name_int_alias[obj]

        return objs

    def get_available_objects_of_types(self, obj_type=[]):
        objs = {}
        for obj, o_type in self._object_type.items():
            if o_type in obj_type:
                objs[obj] = self._name_int_alias[obj]

        return objs

    def create_int_alias(self, old_int_alias={}):
        if len(old_int_alias.values()) == 0:
            next_id = 0
        else:
            next_id = max(old_int_alias.values()) + 1
        for on in self._object_names:
            try:
                self._name_int_alias[on] = old_int_alias[on]
            except KeyError:
                self._name_int_alias[on] = next_id
                next_id += 1

    def lookup_object_names_from_roadmaps(self, obj_types=["block"]):
        for gn in self._rm.keys():
            names = self._rm[gn].get_vertices_for_property(obj_types)
            self._object_names = self._object_names.union(set(names))

    def lookup_rm_nodes_for_objects(self):
        for gn in self._rm.keys():
            for on in self._object_names:
                vertex = self._rm[gn].get_vertex_for_name(on)
                if vertex:
                    self._rm_node[gn][on] = int(str(vertex))

    def lookup_object_types(self):
        for on in self._object_names:
            for gn in self._rm.keys():
                if on in self._rm_node[gn].keys():
                    vertex = self._rm_node[gn][on]
                    self._object_type[on] = self._rm[gn].get_type_for_vertex(vertex)

    def get_pose_for_name(self, on, default=None):
        for gn, node_dict in self._rm_node.items():
            if on in node_dict.keys():
                vert = node_dict[on]
                pose = self._rm[gn].get_eef_pose(vert)
                return pose

    def get_poses(self):
        poses = {}

        for name, alias in self._name_int_alias.items():
            pose = None
            pose = self.get_pose_for_name(name)

            if pose:
                poses[alias] = pose
        return poses

    def get_name_int_alias(self):
        return copy.copy(self._name_int_alias)
