#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import cPickle as pickle
import errno
import os

from prm import RoadMap
import time
import itertools
from graph_tool.all import Vertex
from moveit_msgs.msg import RobotState
import copy
import rospy


class RoadMapClash:
    def __init__(self, groups=[], roadmap_fingerprints={}):
        self.groups = set(groups)
        self.roadmap_fingerprints = {}  # type: dict[str, str]
        self.clashes = {}  # type: dict[tuple(str, str), dict[int, set[int]]]
        self.roadmaps = {}  # type: dict[str, RoadMap]

    def add_roadmap(self, roadmap):
        # type: (RoadMap) -> bool
        assert isinstance(roadmap, RoadMap)
        self.roadmaps[roadmap.get_group_name()] = roadmap

        return isinstance(self.roadmaps[roadmap._group_name], RoadMap)

    def get_clash(self, from_group, to_group):
        try:
            return self.clashes[tuple([from_group, to_group])]
        except KeyError:
            rospy.logerr("No clash from {} to {} available.".format(from_group, to_group))
            return False

    def load_clash(self, file_name="prm_clash", groups=[], file_format=None):
        if file_name == 'empty':
            self.clear_clash()
            rospy.loginfo("Cleared clash.")
            return

        version = None
        if not file_format:
            try:
                if os.path.exists(file_name):
                    with open(file_name, 'rb') as output:
                        loaded_clash = pickle.load(output)
                        version = loaded_clash.file_format_version
                else:
                    with open(file_name + '.pkl', 'rb') as output:
                        loaded_clash = pickle.load(output)
                        version = loaded_clash.file_format_version
            except AttributeError:
                rospy.logwarn("File {} has no version tag. Try to guess content format.".format(file_name))
        else:
            version = file_format

        if version == 0.1:
            with open(file_name, 'rb') as output:
                loaded_clash = pickle.load(output)  # type: RoadMapClash
                # assert isinstance(loaded_clash, RoadMapClash)
                # print type(loaded_clash)
            for group in groups:
                if group not in loaded_clash.groups:
                    rospy.logwarn(
                        "Information about group {} is not available in file {}.pkl.".format(group, file_name))
                    continue
                self.groups.add(group)
                try:
                    self.roadmap_fingerprints[group] = loaded_clash.roadmap_fingerprints[group]
                except KeyError:
                    print "No Roadmap fingerprint for group {} in file {}.pkl.".format(group, file_name)
            for group_combo in loaded_clash.clashes.keys():
                if group_combo[0] in groups and group_combo[-1] in groups:
                    self.clashes[group_combo] = loaded_clash.clashes[group_combo]
                else:
                    print "Clash information for group combo {} in file {}.pkl not used. Not specified in groups: {}".format(
                        group_combo, file_name, groups)

    def save_clash(self, groups=[], file_name="prm_clash"):

        """
        Saves the computed or modified clashes to the disk.
        :param groups: group names for which we want to save the information
        :param file_name: will be saved to <filename-or-path>.pkl
        """

        data_path = rospy.get_param("SolverSetup/data_path", default="")
        data_path = os.path.expanduser(data_path)


        file_format_version = 0.1
        # get a copy of the whole roadmapclash object
        clash_save = copy.deepcopy(self)

        # strip out unwanted clashes
        for group_combo in self.clashes.keys():
            if group_combo[0] not in groups or group_combo[-1] not in groups:
                del clash_save.clashes[group_combo]

        # strips out unwanted groups
        for group in self.groups:
            if group not in groups:
                clash_save.groups.remove(group)

        # strips out unwanted fingerprints
        for group in self.roadmap_fingerprints.keys():
            if group not in groups:
                del clash_save.roadmap_fingerprints[group]

        # strips out all roadmaps
        for group in self.roadmaps.keys():
            del clash_save.roadmaps[group]

        clash_save.file_format_version = file_format_version
        assert clash_save.file_format_version == file_format_version

        filename = os.path.join(data_path, file_name + ".pkl")
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(filename, 'wb') as output:
            pickle.dump(clash_save, output, pickle.HIGHEST_PROTOCOL)

        rospy.loginfo("Saved clash {} to file.".format(filename))

    def clear_clash(self, groups=[]):
        if len(groups) == 0:
            self.clashes = {}
            self.groups = set()
        else:
            del_clash_keys = []
            for group_combo in self.clashes.keys():
                if group_combo[0] in groups or group_combo[-1] in groups:
                    del_clash_keys.append(group_combo)

            for key in del_clash_keys:
                del self.clashes[key]

            for group in groups:
                self.groups.remove(group)

        return

    def check_clash_roadmap_and_copy_fingerprint(self, groups=[]):
        # TODO: change to new clash key tuple
        for group in groups:
            if group in self.clashes.keys() and group in self.roadmaps.keys():
                if len(self.clashes[group].keys()) == self.roadmaps[group].get_number_of_vertices():
                    self.roadmap_fingerprints[group] = self.roadmaps[group].get_fingerprint()
                    self.groups.add(group)

    def verify_clash_roadmap_combinations(self, roadmaps):
        # type: (dict[str, RoadMap]) -> void

        for group in self.groups:
            assert group in [clash_name[0] for clash_name in self.clashes.keys()]
            assert group in roadmaps.keys()

        for clash_key in self.clashes.keys():
            gn_0 = clash_key[0]
            assert len(self.clashes[clash_key].keys()) == roadmaps[gn_0].get_number_of_vertices()
        return

    def build_robot_state(self, prm_1, v1, prm_2, v2):
        # type: (RoadMap, Vertex, RoadMap, Vertex) -> RobotState

        rs1 = prm_1.get_robot_state_for_vertex(v1)  # type: RobotState
        rs2 = prm_2.get_robot_state_for_vertex(v2)  # type: RobotState

        joint_names_2 = prm_2.get_joint_names_of_group()

        rs = rs1
        joint_state_pos_lst = list(rs.joint_state.position)

        for joint_name in joint_names_2:
            i_prm = rs2.joint_state.name.index(joint_name)
            i_rs = rs.joint_state.name.index(joint_name)
            joint_state_pos_lst[i_rs] = rs2.joint_state.position[i_prm]

        rs.joint_state.position = tuple(joint_state_pos_lst)

        return rs

    # TODO: test compute clash refactoring
    def compute_clash_for_prms(self, sv=None):
        start_time = time.time()

        prm_it = itertools.product(self.roadmaps.keys(), self.roadmaps.keys())

        for prm_combo in prm_it:
            prm_combo_l = list(prm_combo)
            prm_combo_l.reverse()
            if prm_combo[0] == prm_combo[1]:
                continue
            elif prm_combo in self.clashes.keys():
                continue
            elif tuple(prm_combo_l) in self.clashes.keys():
                continue
            else:
                prm_1 = self.roadmaps[prm_combo[0]]
                prm_2 = self.roadmaps[prm_combo[1]]
                it = itertools.product(prm_1.vertices(), prm_2.vertices())

            prm_1_comp = {}
            for i in prm_1.vertices():
                prm_1_comp[int(i)] = set()

            prm_2_comp = {}
            for i in prm_2.vertices():
                prm_2_comp[int(i)] = set()

            elapsed_time = time.time() - start_time
            print elapsed_time
            count = 0
            for t in it:  # type: tuple[Vertex, Vertex]
                rs = self.build_robot_state(prm_1, t[0], prm_2, t[-1])
                # TODO: group name for group including all checked groups
                stateVal = sv.getStateValidity(rs, group_name="upperbody")
                # stateVal = self.SV_SRV.getStateValidity(rs, group_name="upperbody")

                if stateVal:
                    prm_1_comp[int(t[0])].add(int(t[-1]))
                    prm_2_comp[int(t[-1])].add(int(t[0]))
                count += 1
                if count > 1000000:
                    break
                if count % 1000 == 0:
                    print count

            print count

            self.clashes[tuple([prm_1.get_group_name(), prm_2.get_group_name()])] = prm_1_comp
            self.clashes[tuple([prm_2.get_group_name(), prm_1.get_group_name()])] = prm_2_comp

            self.roadmap_fingerprints[prm_1.get_group_name()] = prm_1.get_fingerprint()
            self.roadmap_fingerprints[prm_2.get_group_name()] = prm_2.get_fingerprint()

            elapsed_time = time.time() - start_time
            print elapsed_time


if __name__ == "__main__":
    rmc = RoadMapClash()
    # rmc.load_clash()
    rmc.load_clash(groups=["left_arm"], file_name="test_save_load_clash")
