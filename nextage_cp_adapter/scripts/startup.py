#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import copy
import subprocess
from rospkg import ResourceNotFound
import moveit_commander

import rospy
import signal
import sys
import rospkg

from roadmap_tools.prm_factory import RoadMapFactory, RoadMap


class NextageStartupManager:
    def __init__(self):
        self.processes = []  # type: list[subprocess.Popen]

        signal.signal(signal.SIGINT, self.handle_sigint)


    def shutdown_all_processes(self):
        rospy.loginfo("Starting shutdown of {}".format(self.processes))
        self.processes.reverse()
        for proc in self.processes:
            proc_poll = proc.poll()
            if proc_poll is not None:
                continue
            # proc.kill()
            proc.send_signal(signal.SIGINT)
            proc.wait()
            rospy.loginfo("{} is shut down".format(proc))

        rospy.loginfo("Everything is shut down.")

    def handle_sigint(self, sign, frame):
        rospy.loginfo("Starting shutdown.")
        self.shutdown_all_processes()
        # moveit_proc.send_signal(signal.SIGINT)
        # sim_proc.send_signal(signal.SIGINT)
        # moveit_proc.wait()
        # sim_proc.wait()
        #
        # print("all is shut down")
        rospy.loginfo("Shutdown self.")
        sys.exit()

    def start_roscore(self):
        rospy.get_master()

    def start_process(self, package, file):
        pkg = rospkg.RosPack()
        try:
            path = pkg.get_path(package)
            if not isinstance(path, str):
                raise ResourceNotFound
        except ResourceNotFound:
            rospy.logerr("Can't start process. Package {} not found.".format(package))
            return 1
        try:
            if '.launch' in file:
                proc = subprocess.Popen(["roslaunch", package, file])
            else:
                proc = subprocess.Popen(["rosrun", package, file])
            self.processes.append(proc)
        except NameError:
            try:
                proc = subprocess.Popen(["rosrun", package, file])
            except NameError:
                rospy.logerr("Can not rosrun or roslaunch {} from package {}".format(file, package))
                return 2

        return 0

    def move_robot_to_initial_conf(self):
        rc = moveit_commander.RobotCommander()

        mg_right = moveit_commander.MoveGroupCommander("right_arm")
        mg_left = moveit_commander.MoveGroupCommander("left_arm")

        rospy.sleep(2.0)

        pose_right = moveit_commander.PoseStamped()
        pose_right.header.frame_id = 'WAIST'
        pose_right.pose.position.x = 0.0
        pose_right.pose.position.y = -0.5
        pose_right.pose.position.z = 0.15

        pose_right.pose.orientation.x = 0.0
        pose_right.pose.orientation.y = -0.707106781187
        pose_right.pose.orientation.z = 0.0
        pose_right.pose.orientation.w = 0.707106781187

        mg_right.set_pose_target(pose_right, mg_right.get_end_effector_link())
        mg_right.set_start_state_to_current_state()
        plan = mg_right.plan()

        mg_right.execute(plan)

        pose_left = copy.copy(pose_right)
        pose_left.pose.position.y = 0.5

        mg_left.set_pose_target(pose_left, mg_left.get_end_effector_link())
        mg_left.set_start_state_to_current_state()
        plan = mg_left.plan()
        mg_left.execute(plan)





if __name__ == "__main__":

    startupmanager = NextageStartupManager()
    try:
        startupmanager.start_process("nextage_gazebo", "nextage_world.launch")
        rospy.sleep(10.0)
        startupmanager.start_process("nextage_cp_adapter", "robot.launch")


        # wait for moveit to come up
        rospy.wait_for_service("/compute_ik")

        rospy. sleep(5.0)

        startupmanager.move_robot_to_initial_conf()

        startupmanager.start_process("nextage_cp_adapter", "write_solver_params_kawada_base_rm.py")
        rospy. sleep(1.0)

        # create seed robot states
        proc = subprocess.Popen(["rosrun", "roadmap_tools", "save_joint_states.py", "kawada", "off"])
        proc = subprocess.Popen(["rosrun", "roadmap_tools", "save_joint_states.py", "kawada", "ik_seed"])

        rospy.sleep(1.0)

        # check if roadmap and lmdb available
        # prm_left = RoadMapFactory.load_prm_from_database(rospy.get_param("/SolverSetup/ROADMAP_NAMES/left_arm"))

        # startupmanager.start_process("roadmap_planner", "motion_dispatcher.py")
        # startupmanager.start_process("roadmap_planner", "prm_planner_node.py")



    except Exception:
        startupmanager.shutdown_all_processes()
        exit(66)





    # sim_proc = subprocess.Popen(["roslaunch", "nextage_gazebo", "nextage_world.launch"])

    # rospy.sleep(10.0)
    # moveit_proc = subprocess.Popen(["roslaunch", "nextage_moveit_config", "moveit_planning_execution.launch"])

    while True:
        rospy.sleep(1.0)
        print(".")


