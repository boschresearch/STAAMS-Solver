#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from geometry_msgs.msg import TransformStamped, PoseStamped
from roadmap_planner_tools.planner_input_manager import PlannerInputManager
import unittest

from roadmap_planning_common_msgs.msg import OrderedVisitingConstraint, StringList, ConstraintType


class TestPIManager(unittest.TestCase):

    def test_add_poses(self):

        pi_manager = PlannerInputManager()

        trans = TransformStamped()
        trans.child_frame_id = 'board'
        trans.header.frame_id = 'world'
        board_quat = [-0.6646584989424609, 0.7469166744613165, 0.009387090228191897, -0.016013860629187193]
        board_trans = [0.6, 0.3, 0.02]
        trans.transform.translation.x = board_trans[0]
        trans.transform.translation.y = board_trans[1]
        trans.transform.translation.z = board_trans[2]
        trans.transform.rotation.x = board_quat[0]
        trans.transform.rotation.y = board_quat[1]
        trans.transform.rotation.z = board_quat[2]
        trans.transform.rotation.w = board_quat[3]

        pi_manager.add_frame(transform=trans)

        Poses = []
        names = []

        pose = PoseStamped()
        Poses.append(pose)
        names.append('loc_1')

        pose = PoseStamped()
        Poses.append(pose)
        names.append('loc_2')

        pi_manager.add_loc(Poses, names, len(Poses) * ["gluepoint"])

        myOVC = OrderedVisitingConstraint()
        myOVC.name = 'ovc_1'
        loc_names_1 = StringList()
        loc_names_1.values.append('loc_1')
        myOVC.location_names.append(loc_names_1)
        pi_manager.add_ovc([myOVC])

        myOVC_2 = OrderedVisitingConstraint()
        myOVC_2.name = 'ovc_2'
        loc_names_1 = StringList()
        loc_names_1.values.append('loc_2')
        myOVC_2.location_names.append(loc_names_1)
        pi_manager.add_ovc([myOVC_2])

        # ct = Constraints[0]
        pi_manager.add_ovc_ct(constraint_type=ConstraintType.StartsAfterEnd, first_ovcs=['ovc_1'], second_ovcs=['ovc_2'])


        pi_manager.write_planner_input_file('test')
