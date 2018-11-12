#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens

This source code is derived from the dmp_gestures project
(https://github.com/awesomebytes/dmp_gestures)
Copyright (c) 2013, Willow Garage, Inc., licensed under the BSD license,
cf. 3rd-party-licenses.txt file in the root directory of this source tree.
"""

"""
Created on 12/08/14
@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com
This file contains kinematics related classes to ease
the use of MoveIt! kinematics services.
"""

import rospy
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse, \
    GetPositionIK, GetPositionIKRequest, GetPositionIKResponse, \
    GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes, Constraints
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

DEFAULT_FK_SERVICE = "/compute_fk"
DEFAULT_IK_SERVICE = "/compute_ik"
DEFAULT_SV_SERVICE = "/check_state_validity"


class ForwardKinematics():
    """Simplified interface to ask for forward kinematics"""

    def __init__(self):
        rospy.loginfo("Loading ForwardKinematics class.")
        self.fk_srv = rospy.ServiceProxy(DEFAULT_FK_SERVICE, GetPositionFK)
        rospy.loginfo("Connecting to FK service")
        self.fk_srv.wait_for_service()
        rospy.loginfo("Ready for making FK calls")

    def closeFK(self):
        self.fk_srv.close()

    def getFK(self, fk_link_names, joint_names, positions, frame_id='base_link'):
        """Get the forward kinematics of a joint configuration
        @fk_link_names list of string or string : list of links that we want to get the forward kinematics from
        @joint_names list of string : with the joint names to set a position to ask for the FK
        @positions list of double : with the position of the joints
        @frame_id string : the reference frame to be used"""
        gpfkr = GetPositionFKRequest()
        if type(fk_link_names) == type("string"):
            gpfkr.fk_link_names = [fk_link_names]
        else:
            gpfkr.fk_link_names = fk_link_names
        gpfkr.robot_state.joint_state.name = joint_names
        gpfkr.robot_state.joint_state.position = positions
        gpfkr.header.frame_id = frame_id
        # fk_result = GetPositionFKResponse()
        fk_result = self.fk_srv.call(gpfkr)
        return fk_result

    def getCurrentFK(self, fk_link_names, frame_id='base_link'):
        """Get the forward kinematics of a set of links in the current configuration"""
        # Subscribe to a joint_states
        js = rospy.wait_for_message('/joint_states', JointState)
        # Call FK service
        fk_result = self.getFK(fk_link_names, js.name, js.position, frame_id)
        return fk_result


#     def isJointConfigInCollision(self, fk_link_names, joint_names, positions): # This must use InverseKinematics as it has collision avoidance detection
#         """Given a joint config return True if in collision, False otherwise"""
#         fk_result = self.getFK(fk_link_names, joint_names, positions)
#         print fk_result
#         if fk_result.error_code != MoveItErrorCodes.SUCCESS:
#             return False
#         return True


class InverseKinematics():
    """Simplified interface to ask for inverse kinematics"""

    def __init__(self):
        rospy.loginfo("Loading InverseKinematics class.")
        self.ik_srv = rospy.ServiceProxy(DEFAULT_IK_SERVICE, GetPositionIK)
        rospy.loginfo("Connecting to IK service")
        self.ik_srv.wait_for_service()
        rospy.loginfo("Ready for making IK calls")

    def closeIK(self):
        self.ik_srv.close()

    def getIK(self, group_name, ik_link_name, pose, avoid_collisions=True, attempts=None, robot_state=None,
              constraints=None):
        """Get the inverse kinematics for a group with a link a in pose in 3d world.
        @group_name string group i.e. right_arm that will perform the IK
        @ik_link_name string link that will be in the pose given to evaluate the IK
        @pose PoseStamped that represents the pose (with frame_id!) of the link
        @avoid_collisions Bool if we want solutions with collision avoidance
        @attempts Int number of attempts to get an Ik as it can fail depending on what IK is being used
        @robot_state RobotState the robot state where to start searching IK from (optional, current pose will be used
        if ignored)"""
        gpikr = GetPositionIKRequest()
        gpikr.ik_request.group_name = group_name
        if robot_state != None:  # current robot state will be used internally otherwise
            gpikr.ik_request.robot_state = robot_state
        gpikr.ik_request.avoid_collisions = avoid_collisions
        gpikr.ik_request.ik_link_name = ik_link_name
        if type(pose) == type(PoseStamped()):
            gpikr.ik_request.pose_stamped = pose
        else:
            rospy.logerr("pose is not a PoseStamped, it's: " + str(type(pose)) + ", can't ask for an IK")
            return
        if attempts != None:
            gpikr.ik_request.attempts = attempts
        else:
            gpikr.ik_request.attempts = 0
        if constraints != None:
            gpikr.ik_request.constraints = constraints
        ik_result = self.ik_srv.call(gpikr)
        rospy.logwarn("Sent: " + str(gpikr))
        return ik_result


#     def getCurrentIK(self, fk_link_names, frame_id='base_link'):
#         """Get the forward kinematics of a set of links in the current configuration"""
#         # Subscribe to a joint_states
#         js = rospy.wait_for_message('/joint_states', JointState)
#         # Call FK service
#         fk_result = self.getFK(fk_link_names, js.name, js.position, frame_id)
#         return fk_result

class StateValidity():
    def __init__(self):
        rospy.loginfo("Initializing stateValidity class")
        self.sv_srv = rospy.ServiceProxy(DEFAULT_SV_SERVICE, GetStateValidity)
        rospy.loginfo("Connecting to State Validity service")
        self.sv_srv.wait_for_service()
        if rospy.has_param('/play_motion/approach_planner/planning_groups'):
            list_planning_groups = rospy.get_param('/play_motion/approach_planner/planning_groups')
            # Get groups and joints here
            # Or just always use both_arms_torso...
        else:
            rospy.logwarn("Param '/play_motion/approach_planner/planning_groups' not set. We can't guess controllers")
        rospy.loginfo("Ready for making Validity calls")

    def close_SV(self):
        self.sv_srv.close()

    def getStateValidity(self, robot_state, group_name='both_arms_torso', constraints=None):
        """Given a RobotState and a group name and an optional Constraints
        return the validity of the State"""
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        # GetStateValidityResponse()
        # rospy.logwarn("sent: " + str(gsvr))

	# JAN BEHRENS (jan.behrens@de.bosch.com - 2018-10-30): Return a bool instead of the full message
        return result.valid
	################################################################################################


if __name__ == '__main__':
    rospy.init_node("test_kinematics_class")
    rospy.loginfo("Initializing forward kinematics test.")
    fk = ForwardKinematics()
    rospy.loginfo("Current FK:")
    rospy.loginfo(str(fk.getCurrentFK('arm_left_7_link')))
    #     rospy.loginfo("isJointConfigInCollision with all left arm at 0.0 (should be False):")
    #     rospy.loginfo( str(fk.isJointConfigInCollision('arm_left_7_link',
    #                                 ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
    #                                  'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
    #                                  'arm_left_7_joint'],
    #                                  [0.0, 0.0, 0.0,
    #                                   0.0, 0.0, 0.0,
    #                                   0.0]) ))
    #     rospy.loginfo("isJointConfigInCollision with shoulder pointing inwards (should be True):")
    #     rospy.loginfo( str(fk.isJointConfigInCollision('arm_left_7_link',
    #                                 ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
    #                                  'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
    #                                  'arm_left_7_joint'],
    #                                  [-2.0, 0.0, 0.0,
    #                                   0.0, 0.0, 0.0,
    #                                   0.0]) ))

    fk.closeFK()

    rospy.loginfo("Initializing inverse kinematics test.")
    ik = InverseKinematics()
    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position = Point(0.3, -0.3, 1.1)
    ps.pose.orientation.w = 1.0
    rospy.loginfo("IK for:\n" + str(ps))
    args = ["right_arm", "arm_right_7_link", ps]
    rospy.loginfo(str(ik.getIK(*args)))

    ps.pose.position.x = 0.9
    rospy.loginfo("IK for:\n" + str(ps))
    args = ["right_arm", "arm_right_7_link", ps]
    rospy.loginfo(str(ik.getIK(*args)))
    ik.closeIK()