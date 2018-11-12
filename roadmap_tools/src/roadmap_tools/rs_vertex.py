#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import moveit_commander

import moveit_msgs.msg
import moveit_msgs.srv

import cPickle as pickle

import random

from roadmap_tools.robot_info import RobotInfo

import copy

import rospkg
import os


class rs_vertex:
    joint_state_default = None
    rand = random.Random()
    sv = None
    robot_com = None
    rospack = rospkg.RosPack()
    ri = None

    def __init__(self, group_name, joint_names, robot_com=None, robot_name="kawada"):
        # self.rand = random.Random()
        self.joint_names = joint_names  # robot_com.get_joint_names(group_name)
        self.robot_state = None
        self.checked = False  # type: bool
        self.validity_last_check = None
        self.id = None
        self.group_name = group_name
        self.robot_name = robot_name

    def check_vertex_compatibility(self, other, sv=None):
        # type: (rs_vertex) -> bool
        if rs_vertex.robot_com is None:
            rs_vertex.robot_com = moveit_commander.RobotCommander()
        for jn in self.joint_names:
            assert jn not in rs_vertex.robot_com.get_joint_names(other.group_name)
        robot_state = copy.copy(self.robot_state)
        joint_state_position_lst = list(robot_state.joint_state.position)

        for name in other.joint_names:
            i = robot_state.joint_state.name.index(name)
            joint_state_position_lst[i] = other.robot_state.joint_state.position[i]

        robot_state.joint_state.position = tuple(joint_state_position_lst)
        val = rs_vertex.getStateValidity(robot_state, sv=sv)
        # val = rs_vertex.sv.getStateValidity(robot_state, group_name="upperbody")
        return val.valid

    @staticmethod
    def getStateValidity(robot_state, sv=None):
        if rs_vertex.ri is None:
            rs_vertex.ri = RobotInfo.getRobotInfo()  # type: RobotInfo
        groupname = rs_vertex.ri.getCompleteGroup()
        if sv is not None:
            sv.getStateValidity(robot_state, group_name=groupname)
            return
        if rs_vertex.sv is None:
            rs_vertex.sv = rs_vertex.ri.getSV()

        return rs_vertex.sv.getStateValidity(robot_state, group_name=groupname)


    def set_robot_state(self, group_name, robot, set_joint_state=None):
        # load joint state into memory and do not load it from the file every time.
        js = self.get_off_joint_state(robot_name=self.robot_name)
        robot_state = moveit_msgs.msg.RobotState()

        # TODO: get rid of the base state
        js = js.joint_state
        joint_state_position_lst = list(js.position)
        if set_joint_state is None:
            robot_state.joint_state = js
        else:
            robot_state.joint_state = set_joint_state
        for name in self.joint_names:
            if set_joint_state is None:
                i = robot_state.joint_state.name.index(name)
                joint_state_position_lst[i] = robot.joint_map[name].limit.lower + rs_vertex.rand() * (
                    robot.joint_map[name].limit.upper - robot.joint_map[name].limit.lower)
            else:
                try:
                    i2 = set_joint_state.name.index(name)
                except ValueError:
                    print("Joint {} not in state".format(name))
                    continue
                i = i2
                joint_state_position_lst[i] = set_joint_state.position[i2]

        robot_state.joint_state.position = tuple(joint_state_position_lst)

        robot_state.is_diff = False

        self.robot_state = robot_state

    @staticmethod
    def get_off_joint_state(robot_name="kawada", file_name_suffix=None):
        path = rs_vertex.rospack.get_path("roadmap_tools")
        file_name = os.path.join(path, "data", robot_name + "_joint_state")

        if file_name_suffix is not None:
            file_name += "_" + file_name_suffix + ".pkl"
        else:
            file_name += ".pkl"

        if rs_vertex.joint_state_default is None:
            with open(file_name, 'rb') as output:
                js = pickle.load(output)
                rs_vertex.joint_state_default = js
        else:
            js = copy.copy(rs_vertex.joint_state_default)

            print(js)
        return js

    @staticmethod
    def load_off_joint_state(robot_name="kawada", file_name_suffix=""):

        path = rs_vertex.rospack.get_path("roadmap_tools")

        file_name = os.path.join(path, "data", robot_name + "_joint_state")
        file_name += "_" + file_name_suffix + ".pkl"

        with open(file_name, 'rb') as output:
            js = pickle.load(output)
            rs_vertex.joint_state_default = js

            print(js)
        return js

    @staticmethod
    def save_off_joint_state(robot_name="kuka", file_name_suffix=""):
        path = rs_vertex.rospack.get_path("roadmap_tools")
        file_name = os.path.join(path, "data", robot_name + "_joint_state")
        file_name = file_name + "_" + file_name_suffix + ".pkl"
        print(file_name)

        if rs_vertex.robot_com is None:
            rs_vertex.robot_com = moveit_commander.RobotCommander()

        rs = rs_vertex.robot_com.get_current_state()

        print(rs.joint_state)
        with open(file_name, 'wb') as output:
            output.write(pickle.dumps(rs, pickle.HIGHEST_PROTOCOL))
        return
