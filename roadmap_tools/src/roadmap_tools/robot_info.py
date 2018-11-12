#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import rospy
import sys
import inspect

class RobotInfo:
    '''This class provides necessary information to the solver. Users should implement this class for their robot'''

    def __init__(self):
        pass

    @staticmethod
    def getRobotInfo():
        # type: (None) -> RobotInfo
        ROBOT_INFO_CLASS = rospy.get_param("SolverSetup/ROBOT_INFO_CLASS", default=None)
        if ROBOT_INFO_CLASS is None:
            rospy.logerr("Parameter [SolverSetup/ROBOT_INFO_CLASS] is not provided. Can not import class")
            assert False

        # check that provided class and package names is string
        assert isinstance(ROBOT_INFO_CLASS, str)

        moduledotclass = ROBOT_INFO_CLASS.split(".")
        if len(moduledotclass) != 2:
            rospy.logerr("Parameter [SolverSetup/ROBOT_INFO_CLASS] string is not in the format module.class: {}".format(ROBOT_INFO_CLASS))
            assert False

        if moduledotclass[0] not in sys.modules.keys():
            rospy.logerr("Module {} not known. Make sure to install the package containing your Robot Info script".format(moduledotclass[0]))
            rospy.loginfo("Known issue: Although the module is not found in the system, the import works - hopefully.")
            # TODO: find why nextage_cp_adapter is not installed properly, but we can anyway use it
            # assert False

        try:
            exec("import " + ROBOT_INFO_CLASS + " as ri")

            for name, obj in inspect.getmembers(ri):
                if inspect.isclass(obj):
                    if name == "RobotInfo":
                        continue
                    class_name = name
                    rospy.loginfo("Found class: {}".format(obj))
                    break
            exec("robot_info = ri." + class_name + "()")
        except NameError:
            rospy.logerr("Parameter [SolverSetup/ROBOT_INFO_CLASS] is not provided. Can not import class")

        try:
            groups = robot_info.getGroups()
            assert isinstance(groups, list)
            for gn in groups:
                assert isinstance(gn, str)
        except AttributeError:
            rospy.logerr("RobotInfo object has not all required Methods.")
            assert False

        return robot_info

    def getSV(self):
        pass

    def getIK(self):
        pass

    def getFK(self):
        pass

    def getRoots(self, now):
        pass

    def getStaticRobotTransforms(self, now):
        pass

    def getGroups(self):
        pass

    def getCompleteGroup(self):
        pass

    def getBaseFrame(self):
        pass

    def getEndEffector(self, group):
        # group2ee = {"r1_arm": "r1_ee", "r2_arm": "r2_ee"}
        # return group2ee[group]
        pass
