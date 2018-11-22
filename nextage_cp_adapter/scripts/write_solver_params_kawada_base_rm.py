#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""


import rospy
import rosparam


if __name__ == "__main__":

    param_ns = 'SolverSetup'
    rospy.set_param('SolverSetup/ROBOT_NAME', "kawada")
    rospy.set_param('SolverSetup/STEPS_MAX', 50)
    rospy.set_param('SolverSetup/TIME_MAX', 10000)
    rospy.set_param('SolverSetup/LUBY_RESTART_CONSTANT', 5)
    rospy.set_param('SolverSetup/BASE_SEED', 5)
    rospy.set_param('SolverSetup/SEED', rospy.get_param('~setup/BASE_SEED', default=5))
    rospy.set_param('SolverSetup/solver_time', 100000)
    # rospy.set_param(param_ns + '/ROADMAP_NAMES', {'r2_arm': 'prm_r2_arm_2018-05-30 15:09:33.898024', 'r1_arm': 'prm_r1_arm_2018-05-30 14:57:07.705629'})
    # rospy.set_param(param_ns + '/CLASH_NAME', "clash_1528066416")
    # rospy.set_param(param_ns + '/GROUP_NAMES', ['r2_arm', 'r1_arm'])
    # rospy.set_param(param_ns + '/COLLISION', True)
    # rospy.set_param(param_ns + '/EXECUTE_MOTION', True)
    # for kawada robot
    rospy.set_param(param_ns + '/ROADMAP_NAMES', {'right_arm': 'prm_right_arm_2018-09-06 10:21:01.967323', 'left_arm': 'prm_left_arm_2018-09-06 10:21:02.392609'})
    rospy.set_param(param_ns + '/CLASH_NAME', "clash_1536222850.36.pkl")
    rospy.set_param(param_ns + '/GROUP_NAMES', ['right_arm', 'left_arm'])
    rospy.set_param(param_ns + '/BASE_FRAME', 'WAIST')
    rospy.set_param(param_ns + '/COLLISION', True)
    rospy.set_param(param_ns + '/EXECUTE_MOTION', True)

    rospy.set_param(param_ns + '/WORKSPACE/X_MIN', 0.0)
    rospy.set_param(param_ns + '/WORKSPACE/X_MAX', 1.2)
    rospy.set_param(param_ns + '/WORKSPACE/X_STEP', 0.1)

    rospy.set_param(param_ns + '/WORKSPACE/Y_MIN', -0.8)
    rospy.set_param(param_ns + '/WORKSPACE/Y_MAX', 0.8)
    rospy.set_param(param_ns + '/WORKSPACE/Y_STEP', 0.1)

    rospy.set_param(param_ns + '/WORKSPACE/Z_MIN', 0.3)
    rospy.set_param(param_ns + '/WORKSPACE/Z_MAX', 0.3)
    rospy.set_param(param_ns + '/WORKSPACE/Z_STEP', 0.1)



    rospy.set_param("SolverSetup/ROBOT_INFO_CLASS", "nextage_cp_adapter.kawada_robot_info")

    rospy.set_param("SolverSetup/data_path", "~/OVC_DATA/")

    rosparam.dump_params("planner_params.yaml", "SolverSetup/", True)