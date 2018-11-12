#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from roadmap_tools.roadmap_clash import RoadMapClash
from roadmap_tools.kinematics_interface_kuka import StateValidity
from roadmap_tools.prm_factory import RoadMapFactory
import time


def create_clash(group_names, roadmap_names):
    sv = StateValidity()
    rmc = RoadMapClash(group_names)
    # prm_factory = RoadMapFactory()
    for group_name in group_names:
        r = RoadMapFactory.load_prm_from_database(roadmap_names[group_name])
        rmc.add_roadmap(r)

    rmc.compute_clash_for_prms(sv=sv)
    clash_name = "clash_" + str(time.time()).split(".")[0]
    rmc.save_clash(groups=group_names, file_name=clash_name)

    # rmc.save_clash(groups=["left_arm", "right_arm"], file_name="demo_build_clash")

    print roadmap_names, clash_name
    return clash_name



if __name__ == "__main__":

    rm_names = {'r2_arm': 'prm_r2_arm_2018-05-30 15:09:33.898024', 'r1_arm': 'prm_r1_arm_2018-05-30 14:57:07.705629'}
    clash = create_clash(rm_names.keys(), rm_names)

    print(clash)
