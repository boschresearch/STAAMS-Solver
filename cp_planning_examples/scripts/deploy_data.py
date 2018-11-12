#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import shutil
import os
import rospy
import rospkg

from roadmap_tools.prm_factory import RoadMapFactory

if __name__ == "__main__":
    # check if data folder is present

    DATA_DIR = "OVC_DATA"
    home = os.path.expanduser("~")
    data_path = os.path.join(home, DATA_DIR)

    if not os.path.exists(data_path):
        os.makedirs(data_path)

    # find data to be copied
    rospack = rospkg.RosPack()
    PATH_planning_examples = rospack.get_path("cp_planning_examples")
    shutil.copy(os.path.join(PATH_planning_examples, "data", "clash_1536222850.36.pkl"), data_path)

    # save the example roadmaps to the lmdb
    rm_path = os.path.join(PATH_planning_examples, "data", "prm_left_arm_2018-09-06 10:21:02.392609.pkl")
    rm = RoadMapFactory.load_from_file(rm_path)
    RoadMapFactory.save_prm_to_database(rm, True, "")

    rm_path = os.path.join(PATH_planning_examples, "data", "prm_right_arm_2018-09-06 10:21:01.967323.pkl")
    rm = RoadMapFactory.load_from_file(rm_path)
    RoadMapFactory.save_prm_to_database(rm, True, "")




