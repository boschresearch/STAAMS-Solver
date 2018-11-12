#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from roadmap_tools.rs_vertex import rs_vertex
import sys
import rospy

if __name__ == "__main__":
    '''
    This script records the current joint state and saves it as <robot_name>_joint_state_<suffix>.pkl.
    
    The implemented robot names are "kuka" and "kawada" and the expected suffix is "ik_seed" and "off".
    
    Bring the robot into the desired joint state and call the script
    
    rosrun roadmap_tools save_joint_states.py robot_name:=kawada suffix:=ik_seed
    '''

    print(sys.argv)

    if len(sys.argv) < 3:
        print("usage: save_joint_states.py robot_name suffix")
    else:
        print("robot_name is {}".format(sys.argv[1]))
        myargv = rospy.myargv(argv=sys.argv)
        print(myargv)
        robot_name = myargv[1]
        suffix = myargv[2]
        print(robot_name)
        print(suffix)
        rs_vertex.save_off_joint_state(robot_name=robot_name, file_name_suffix=suffix)
