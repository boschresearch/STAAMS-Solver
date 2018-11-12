#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import shutil
from datetime import datetime
import os
import rospy


class ovc_experiment:
    def __init__(self, exps_folder="/cp_planning_experiments/", template_folder="/../../../../share/roadmap_planner/template/", name="", scene=""):
        exp_host = "/OVC_Experiment/"

        subfolder = "run_{:04d}".format(rospy.get_param(exp_host + 'setup/RUN_ID', default=9999))
        self.exps_folder = exps_folder + subfolder + "/"
        scene = rospy.get_param(exp_host + 'setup/scene', default="")
        if len(scene) > 0:
            self.exps_folder += "/" + scene + "/"
        self.name = name + '_' + str(datetime.now())
        home = os.path.expanduser("~")
        self.destination_folder = home + self.exps_folder + self.name
        dir_path = os.path.dirname(os.path.realpath(__file__))
        shutil.copytree(dir_path + template_folder + "experiment_template", self.destination_folder)

        self.solutions = []
        self.solver_stats = []
        self.motion_requests = []
        self.exp_setup = None
        self.solution_times = []
        self.solution_quality = []

        self.cur_sol_folder = None

    def add_solution(self, solution):
        self.solutions.append(solution)
        folder = self.destination_folder + "/solutions/" + '{:04d}/'.format(len(self.solutions))
        self.cur_sol_folder = folder

        if not os.path.exists(folder):
            os.makedirs(folder)

        sol_str = ""
        for k, v in self.solutions[-1].items():
            if v is None or k is None:
                print "I'm here!"

            # print k
            # print v
            s = k
            s += v.__str__()
            s += "\n"

            sol_str += s

        with open(self.cur_sol_folder + "sol.txt", "w") as text_file:
            text_file.write(sol_str)
            # TODO: make textfile useful, i.e. print objects separately

        return

    def add_extra_sol_file(self, src):
        s = "/" + src.split("///")[-1]
        if self.cur_sol_folder:
            shutil.copy(s, self.cur_sol_folder)
        else:
            print "No solution available. Copying to {}/misc.".format(self.destination_folder)
            shutil.copy(s, self.destination_folder + "/misc/")

    def add_solver_stats(self, stats):
        self.solver_stats.append(stats)

    def save_exp_setup(self, setup):
        self.exp_setup = setup
        txt = open(self.destination_folder + "/exp.info", "w")

        txt.write("Experiment parameter: \n")

        txt.write(str(setup))

        txt.close()

    def add_motion_requests(self, req=[]):
        self.motion_requests.append(req)

    def add_time_to_solution(self, time=None):
        if time:
            self.solution_times.append(time)

    def add_solution_quality(self, q):
        if q:
            self.solution_quality.append(q)



