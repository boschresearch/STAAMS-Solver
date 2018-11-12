#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import unittest

import rostest


class TestRoadmap(unittest.TestCase):

    def test_create_roadmap_with_parameters(self):
        from roadmap_tools.prm import RoadMap
        active_joints = ['RARM_JOINT0',
                         'RARM_JOINT1',
                         'RARM_JOINT2',
                         'RARM_JOINT3',
                         'RARM_JOINT4',
                         'RARM_JOINT5']
        rm = RoadMap(group_name="right_arm", base_frame="WAIST", eef_link="RARM_JOINT5_Link", active_joints=active_joints)

        self.assertEqual(rm.get_joint_names_of_group(), active_joints)

    def test_add_nodes_and_vertices(self):
        from roadmap_tools.prm import RoadMap
        from roadmap_tools.prm import RobotState
        active_joints = ['RARM_JOINT0',
                         'RARM_JOINT1',
                         'RARM_JOINT2',
                         'RARM_JOINT3',
                         'RARM_JOINT4',
                         'RARM_JOINT5']
        rm = RoadMap(group_name="right_arm", base_frame="WAIST", eef_link="RARM_JOINT5_Link", active_joints=active_joints)

        v1 = rm.add_vertex(RobotState(), "my_node_1", "test_node")
        v2 = rm.add_vertex(RobotState(), "my_node_2", "test_node")
        e = rm.add_edge(v1, v2)

        self.assertEqual(rm.get_number_of_vertices(), 2)
        self.assertEqual(rm.get_number_of_edges(), 1)


PKG = 'roadmap_tools'
NAME = 'test_prm'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestRoadmap)
