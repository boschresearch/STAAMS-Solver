#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import rospy
from roadmap_tools.prm_factory import RoadMapFactory, RoadMap

'''
example how to get a Roadmap from the database
'''

if __name__ == "__main__":
    prm_left_name = rospy.get_param("/SolverSetup/ROADMAP_NAMES/left_arm", None)
    assert prm_left_name is not None
    filename = RoadMapFactory.export_from_db_to_file(prm_left_name)
    # shutil.move(filename, "../data/")

    prm_right_name = rospy.get_param("/SolverSetup/ROADMAP_NAMES/right_arm", None)
    assert prm_right_name is not None
    filename = RoadMapFactory.export_from_db_to_file(prm_right_name)
    # shutil.move(filename, "../data/")

