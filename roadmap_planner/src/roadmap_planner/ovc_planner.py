#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import itertools
import random

import numpy as np

from ortools.constraint_solver import pywrapcp

from roadmap_planner.BindIntervalsGroupCt import BindIntervalsGroupCt
from roadmap_planner.solution_saving import *

from roadmap_tools.SceneObjectSearchMapping import SceneObjectSearchMapping


from CustomConstraints import IntervalClashCt, Edge2IntervalDurationCt, PathFeasibleCt, \
    MoveOnGraphCt, InGoal2StatesCt, EndBound2UpperBoundCt, ConnectedComponentCt, BindIntervalsCt, IndexedEqualityCt, \
    HorizonConfigurationCt, BrushArmAssignment2PnPTasks, IndexedEqualityVarCt, ConnectSameGroupCt

from ordered_visiting_ct_var import OrderedVisitingConstraintVar
from roadmap_planner.solution_saving import *

import rospy
from roadmap_planning_common_msgs.srv import BuildMotionAddTrajectory, BuildMotionAddTrajectoryRequest, \
    BuildMotionAddTrajectoryResponse
from std_srvs.srv import EmptyRequest, Empty, EmptyResponse

from roadmap_tools.prm_factory import RoadMapFactory
from roadmap_tools.roadmap_clash import RoadMapClash
from roadmap_tools.prm import RoadMap
from datetime import datetime

# import plotly.plotly as py
import plotly.offline as py
import plotly.figure_factory as ff


class OvcPlanner:
    def __init__(self):
        # n, prm_left, prm_right, clash_l, clash_r

        self.roadmaps = {}  # type: dict[str, RoadMap]
        self.clash = RoadMapClash()  # type: RoadMapClash
        self.sosm = SceneObjectSearchMapping()  # Type: SceneObjectSearchMapping

        self.solver = pywrapcp.Solver("CP_Motion_Planner")  # type: pywrapcp.Solver

        self.intvar_lists = {}
        self.intervalvar_lists = {}

        self.refinement_intervals = {}

        self.resource_intervals = {}
        self.ordered_visiting_cst_vars = []

        self.steps_max = -1
        self.time_max = -1

        self.db = None
        self.search_monitors = None

        # self.var_dict = {}  # type: dict[str, list[pywrapcp.IntVar]]

    def get_refine_interval(self, name):
        try:
            i_s = self.refinement_intervals[name]
        except KeyError:
            i_s = self.solver.IntervalVar(0, 10000, 0, 10000, 0, 10000, False, name)  # type: pywrapcp.IntervalVar
            self.refinement_intervals[name] = i_s
        return i_s

    def build_manipulation_model(self, steps_max, time_max, collision_on=True):
        self.steps_max = steps_max
        self.time_max = time_max
        self.create_basic_movement_variables(steps_max, time_max)
        solver = self.solver

        if collision_on:
            # Adding the clash constraint on each pair of components
            it = itertools.product(self.roadmaps.keys(), self.roadmaps.keys())
            for group_combo in it:
                if group_combo[0] == group_combo[-1]:
                    continue
                c1 = self.intvar_lists["c_" + group_combo[0]]
                c2 = self.intvar_lists["c_" + group_combo[-1]]
                i1 = self.intervalvar_lists["i_" + group_combo[0]]
                i2 = self.intervalvar_lists["i_" + group_combo[-1]]
                clash = self.clash.clashes[group_combo]
                rm1 = self.roadmaps[group_combo[0]]
                rm2 = self.roadmaps[group_combo[-1]]

                # Uncomment for path interval clash ct
                # from interval_clash_ct_path import IntervalClashPathCt
                # solver.Add(IntervalClashPathCt(solver, c1, c2, i1, i2, rm1, rm2, clash))  # Todo: uncomment!!!


                from roadmap_planner.interval_clash_path_refine_ct import IntervalClashPathRefineCt
                solver.Add(IntervalClashPathRefineCt(solver, c1, c2, i1, i2, rm1, rm2, clash, self))


                # solver.Add(IntervalClashCt(solver, c1, c2, i1, i2, rm1, rm2, clash))  # Todo: uncomment!!!

        # solver.Add(IntervalClashCt(solver, c_left, c_right, i_left, i_right, clash_l))
        # solver.Add(IntervalClashCt(solver, c_right, c_left, i_right, i_left, clash_r))

        # i mod 2:
        # 0: nodes
        # 1: edges

        for group in self.roadmaps.keys():
            conf_vars, interval_vars = self.get_vars_for_group(group)

            # add duration constraints for the intervals
            for i in range(0, len(interval_vars)):
                if np.mod(i, 2) == 0:
                    # Duration constraint for node intervals
                    interval_vars[i].SetDurationRange(0, time_max)
                else:
                    # Duration constraint for edge intervals
                    ct = Edge2IntervalDurationCt(solver, interval_vars[i], conf_vars[(i - 1) / 2],
                                                 conf_vars[(i - 1) / 2 + 1], self.roadmaps[group])
                    solver.Add(ct)  # Todo: uncomment!!!

            # chain all the intervals of one group
            for i1, i2 in zip(range(0, len(interval_vars) - 1, 1), range(1, len(interval_vars), 1)):
                solver.Add(interval_vars[i1].EndsAtStart(interval_vars[i2]))

            # let the intervals start at 0
            interval_vars[0].SetStartRange(0, 0)  # StartExpr().SetValue(0)

            # let the last interval and be a lower bound for the make span
            solver.Add(self.intervalvar_lists["make_span_interval"][0].EndsAfterEnd(interval_vars[-1]))

            # TODO: test to add all interval variables to opt_time_goal
            solver.Add(interval_vars[-1].EndExpr() <= self.intvar_lists["opt_time_goal"][0])


            # use sum of durations as lower bound of opt_time_goal
            # dur_list = [var.DurationExpr() for i, var in enumerate(interval_vars)]
            # solver.Add(solver.Sum(dur_list) <= self.intvar_lists["opt_time_goal"])

        self.intervalvar_lists["make_span_interval"][0].SetStartRange(0, 0)

        # connect optimization variable and Intervals
        solver.Add(self.intervalvar_lists["make_span_interval"][0].EndExpr() <= self.intvar_lists["opt_time_goal"][0])

        # Adding Constraint to make configurations adhere to roadmap
        for group in self.roadmaps.keys():
            conf_vars, interval_vars = self.get_vars_for_group(group)

            # solver.Add(PathFeasibleCt(solver, conf_vars, self.roadmaps[group]))  # Todo: uncomment!!!

        # collect the last interval of each group and constrain them to end at the same time
        last_intervals = []
        for group in self.roadmaps.keys():
            conf_vars, interval_vars = self.get_vars_for_group(group)

            last_intervals.append(interval_vars[-1])

        it = itertools.combinations(last_intervals, 2)
        for end_intervals in it:
            solver.Add(end_intervals[0].EndsAtEnd(end_intervals[-1]))

        solver.Add(
            EndBound2UpperBoundCt(solver, last_intervals, self.intvar_lists["opt_time_goal"][0]))  # Todo: uncomment!!!

        # # not allow two consecutive same values for configurations
        # for group in self.roadmaps.keys():
        #     interval_vars = self.intervalvar_lists["i_" + group]  # type: list[pywrapcp.IntervalVar]
        #     conf_vars = self.intvar_lists["c_" + group]
        #
        #     for i1 in range(0, len(conf_vars) - 2):
        #         i2 = i1 + 1
        #         solver.Add(conf_vars[i1] != conf_vars[i2])

    def create_basic_movement_variables(self, steps_max, time_max):
        """
        Creates the basic variables to represent the locations of the manipulators and the
        associated interval variables representing the traveling and waiting times.
        conf index        0   1   2   3
                          o---o---o---o
        interval index    0 1 2 3 4 5 6
        conf -> waiting_interval:   i_int = i_conf * 2
        conf -> traveling_interval: i_int = i_conf * 2 +/-1

        :param time_max: maximal ending time for the last interval. this value is NOT in seconds, but gets scaled later.
        :param steps_max: number of variables to create. some may be disabled while solving
        """
        # create the configuration variables for every roadmap and max steps and save them
        # the variable dictionary as c_<group_name>, e.g. c_left_arm
        number_format = len(str(2 * steps_max - 1))
        for key in self.roadmaps.keys():
            n_confs = self.roadmaps[key].get_number_of_vertices()
            conf_domain_max = 100000
            name = 'c_' + key
            conf_var = [self.solver.IntVar(0, conf_domain_max, 'c_' + key + "_{:04d}".format(i)) for i in range(steps_max)]
            self.intvar_lists[name] = conf_var

            horizon = [self.solver.IntVar(0, steps_max, "horizon_{}".format(key))]
            name = "horizon_" + key
            self.intvar_lists[name] = horizon

            ct = HorizonConfigurationCt(self.solver, conf_var, horizon[0])
            self.solver.Add(ct)

        # create a series of travel and waiting intervals for each component (with possible states defined as roadmap)
        for key in self.roadmaps.keys():
            int_var = [self.solver.IntervalVar(0, time_max,  # type: list(IntervalVar) # start time
                                               0, time_max,  # duration
                                               0, time_max,  # end time
                                               False,  # optional
                                               "Interval_{}_{:04d}".format(key, i))
                       for i in range(0, 2 * steps_max - 1)]
            name = 'i_' + key
            self.intervalvar_lists[name] = int_var

        # create interval representing the makespan
        make_span_interval = [self.solver.IntervalVar(0, time_max,  # type: list(IntervalVar) # start time
                                                     0, time_max,  # duration
                                                     0, time_max,  # end time
                                                     False,  # optional
                                                      "Interval_makespan")]
        name = "make_span_interval"
        self.intervalvar_lists[name] = make_span_interval

        # integer variable derived from the makespan as optimization criteria
        opt_time_goal = [self.solver.IntVar(0, time_max, "Goal_time")]
        name = "opt_time_goal"
        self.intvar_lists[name] = opt_time_goal

    def register_resource_demand(self, interval_var, res="left_arm_gripper"):
        try:
            self.resource_intervals[res].append(interval_var)
        except KeyError:
            self.resource_intervals[res] = []
            self.resource_intervals[res].append(interval_var)

    def get_range_of_list(self, rlist=[]):
        return min(rlist), max(rlist)

    def register_order(self, interval_list=[]):
        if len(interval_list) < 2:
            return
        for i1, i2 in zip(interval_list[:-1], interval_list[1:]):
            assert isinstance(i1, pywrapcp.IntervalVar)
            assert isinstance(i2, pywrapcp.IntervalVar)
            self.solver.Add(i2.StartsAfterEnd(i1))

    def build_constraints(self):
        # TODO: check if we still need this function
        from roadmap_planner.CustomConstraints import SameCompOVCConnectAllDifferent

        if self.ordered_visiting_cst_vars.__len__() > 1:
            ct = SameCompOVCConnectAllDifferent(self.solver, self.ordered_visiting_cst_vars)
            self.solver.Add(ct)

        for key, interval_list in self.resource_intervals.items():
            # do nothing if there is only one interval for a resource
            if len(interval_list) <= 1:
                continue
            it = itertools.combinations(interval_list, 2)
            for interval_tuple in it:
                print interval_tuple
                ct = self.solver.TemporalDisjunction(interval_tuple[0], interval_tuple[1])
                self.solver.Add(ct)

    def makeOVC_connect_vars_allDifferent(self):
        ovc_connect_group_dict = {}
        for ovc in self.ordered_visiting_cst_vars:
            if ovc._execution_group.Bound():
                group = ovc._execution_group.Value()
                try:
                    ovc_connect_group_dict[group] += ovc._conf_connect_vars
                except KeyError:
                    ovc_connect_group_dict[group] = []
                    ovc_connect_group_dict[group] += ovc._conf_connect_vars
                    # print(ovc_connect_group_dict[group])

        for connect_var_list in ovc_connect_group_dict.values():
            ct = self.solver.AllDifferent(connect_var_list)
            self.solver.Add(ct)

    def make_ovc_loc_alldifferent_except_count(self, ovc_list=[], loc_pos=None, visit_vals=None):
        # Todo: check if this works

        if len(ovc_list) < 2:
            return
        if loc_pos is None:
            return
        if visit_vals is None:
            return

        loc_vars = []

        from collections import Counter
        val_hist = Counter(visit_vals)

        for ovc in ovc_list:
            loc_vars.append(ovc._conf_values[loc_pos])

        lb = len(val_hist.values()) * [0]

        ct = self.solver.Distribute(loc_vars, val_hist.keys(), lb, val_hist.values())
        self.solver.Add(ct)

    def make_ovc_loc_alldifferent(self, ovc_list=[], loc_pos=None):
        if len(ovc_list) < 2:
            return
        if loc_pos is None:
            return

        loc_vars = []

        # for ovc in ovc_list:
        #     loc_vars.append(ovc._conf_values[loc_pos])

        for ovc in ovc_list:
            loc_vars.append(ovc._loc_values[loc_pos])

        self.solver.Add(self.solver.AllDifferent(loc_vars))

    def make_ovc_monotonous(self, ovc_list=[]):
        if len(ovc_list) < 2:
            return

        for ovc1, ovc2 in zip(ovc_list[0:-2], ovc_list[1:-1]):
            self.solver.Add(ovc1._conf_connect_vars[-1] < ovc2._conf_connect_vars[0])
            # self.solver.Add(ovc2._spanning_visit_interval.StartsAfterEnd(ovc1._spanning_visit_interval))

    def addOVC(self, group, locations=[]):
        '''
        This function adds an Ordered visiting constraint for a fixed group and order.
        :param group:
        :param locations:
        :return:
        '''
        name = "{:04d}".format(len(self.ordered_visiting_cst_vars))
        ovc = OrderedVisitingConstraintVar(self.solver, len(locations), self.time_max, self.steps_max,
                                           self.sosm.get_groups().keys(), self.roadmaps[group].get_number_of_vertices(),
                                           name=name)  # Type: OrderedVisitingConstraintVar
        ovc._execution_group.SetValue(self.sosm.get_groups()[group])

        for l in enumerate(locations):
            ovc._conf_values[l[0]].SetValues(l[-1])

        conf_vars, interval_vars = self.get_vars_for_group(group)

        assert len(ovc._conf_connect_vars) == len(ovc._visit_interval_vars)
        ct = BindIntervalsCt(self.solver, interval_vars, ovc._conf_connect_vars, ovc._visit_interval_vars)
        self.solver.Add(ct)
        for k, connect_var in enumerate(ovc._conf_connect_vars):
            if not ovc._conf_values[k].Bound():
                ct = IndexedEqualityVarCt(self.solver, conf_vars, connect_var, ovc._conf_values[k])
            else:
                ct = IndexedEqualityCt(self.solver, conf_vars, connect_var, ovc._conf_values[k].Value())
            self.solver.Add(ct)

        self.ordered_visiting_cst_vars.append(ovc)

        return ovc

    def addFreeOVC(self, groups=[], locations=[], ranges=None):
        '''
        This function adds an Ordered visiting constraint for a free group and order.
        :param group: the robot components that should be considered fot the task
        :param locations: a list of lists with locations (integer)
        :return: the ovc object
        '''

        assert type(locations) == list
        for l in locations:
            assert type(l) == list

        loc_mapping = self.sosm.get_alias_to_poses()  # type: dict[str, dict[int, int]]
        # nr_locs = max([max(loc_mapping[group].keys()) for group in groups])
        nr_confs = max([self.roadmaps[group].get_number_of_vertices() for group in groups])
        name = "{:04d}".format(len(self.ordered_visiting_cst_vars))
        self.print_model()
        ovc = OrderedVisitingConstraintVar(self.solver, len(locations), self.time_max, self.steps_max,
                                           nr_confs,
                                           loc_mapping=loc_mapping,
                                           name=name,
                                           group_mapping=self.sosm.get_groups(),
                                           ranges=ranges)  # Type: OrderedVisitingConstraintVar
        ovc._execution_group.SetValues([self.sosm.get_groups()[group] for group in groups])

        for l in enumerate(locations):
            # ovc._conf_values[l[0]].SetValues(l[-1])
            ovc._loc_values[l[0]].SetValues(l[-1])

        cts = {}

        for gn, gv in self.sosm.get_groups().items():
            conf_vars, interval_vars = self.get_vars_for_group(gn)
            cts[gv] = [BindIntervalsCt(self.solver, interval_vars, ovc._conf_connect_vars, ovc._visit_interval_vars)]

        ct = BindIntervalsGroupCt(self.solver, ovc._execution_group, cts)
        self.solver.Add(ct)
        assert len(ovc._conf_connect_vars) == len(ovc._visit_interval_vars)

        cts = {}
        for gn, gv in self.sosm.get_groups().items():
            conf_vars, interval_vars = self.get_vars_for_group(gn)
            cts[gv] = []

            for k, connect_var in enumerate(ovc._conf_connect_vars):
                if not ovc._conf_values[k].Bound():
                    cts[gv].append(IndexedEqualityVarCt(self.solver, conf_vars, connect_var, ovc._conf_values[k]))
                else:
                    cts[gv].append(IndexedEqualityCt(self.solver, conf_vars, connect_var, ovc._conf_values[k].Value()))
                    # self.solver.Add(ct)
        self.solver.Add(BindIntervalsGroupCt(self.solver, ovc._execution_group, cts))

        self.ordered_visiting_cst_vars.append(ovc)

        return ovc

    def add_visiting_Ct(self, group, locations=[], start=False, goal=False, state_index=None, earliest=-1, latest=-1):
        conf_vars, interval_vars = self.get_vars_for_group(group)
        if start:
            conf_vars[0].SetValues(locations)
            return
        if goal:
            conf_vars[-1].SetValues(locations)
            return
        if state_index is not None:
            try:
                conf_vars[state_index].SetValues(locations)
                return
            except KeyError:
                print("invalid state_index: {}. Len(conf_vars) is {}.".format(state_index, len(conf_vars)))
                assert False
                return
        if earliest == -1 and latest == -1:
            for loc in locations:
                count_var = self.solver.IntVar(1, self.steps_max)
                ct = self.solver.Count(conf_vars, loc, count_var)
                # ct = solver.Distribute(c_left, pos_visit_left, len(pos_visit_left) * [1], len(pos_visit_left) * [steps])
                self.solver.Add(ct)
            return
        if earliest in range(self.steps_max) and latest in range(self.steps_max) and earliest < latest and len(
                locations) == 1:
            conf_vars_select = []
            for step in range(earliest, latest):
                conf_vars_select.append(conf_vars[step])

            count_var = self.solver.IntVar(1, self.steps_max)
            ct = self.solver.Count(conf_vars_select, locations[0], count_var)
            self.solver.Add(ct)

    def define_decision_builders(self):
        solver = self.solver

        intervalvars = []
        # db_0a = solver.Phase([self.intvar_lists["opt_time_goal"][0]],
        #                      solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MAX_VALUE)

        conf_vars = []
        interval_vars = []
        for group in self.roadmaps.keys():
            c_vars, i_vars = self.get_vars_for_group(group)
            conf_vars += c_vars
            interval_vars += i_vars
        intervalvars += interval_vars

        # will be filled with decision builders
        decision_builders = []

        visit_intervals = []
        visit_configurations = []
        for group in self.roadmaps.keys():
            try:
                visit_intervals += self.intervalvar_lists['visit_i_' + group]
                visit_configurations += self.intvar_lists['visit_c_' + group]

                intervalvars += visit_intervals
            except KeyError:
                print "There are no ordered visit constraints for group {}".format(group)

        have_visit_ct = False
        if len(visit_configurations) > 0:
            have_visit_ct = True

        if have_visit_ct:
            var_list = []
            for l in visit_configurations:
                for var in l:
                    var_list.append(var)
            db_visit_confs = solver.Phase(var_list, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)

        have_visiting_ct_var = False
        try:
            if len(self.ordered_visiting_cst_vars) > 0:
                have_visiting_ct_var = True
        except AttributeError:
            pass

        if have_visiting_ct_var:
            var_list = []
            for l in self.ordered_visiting_cst_vars:
                for var in l._conf_connect_vars:
                    var_list.append(var)
            db_visiting_ct_var = solver.Phase(var_list, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)

            resource_intervals = []
            for var in self.ordered_visiting_cst_vars:
                resource_intervals.append(var._spanning_visit_interval)


            var_list = []
            for l in self.ordered_visiting_cst_vars:  # type: OrderedVisitingConstraintVar
                for var in l._visit_interval_vars:
                    var_list.append(var)
                for var in [l._spanning_visit_interval]:
                    var_list.append(var)
            db_visiting_intervals_ct_var = solver.Phase(var_list + resource_intervals, solver.INTERVAL_DEFAULT)
            intervalvars += var_list

        # db_1 = solver.Phase(self.intvar_lists["opt_time_goal"] + conf_vars,
        #                     solver.CHOOSE_MIN_SIZE, solver.ASSIGN_RANDOM_VALUE)

        db_1 = solver.Phase(conf_vars,
                            solver.CHOOSE_MIN_SIZE, solver.ASSIGN_RANDOM_VALUE)

        # db_1b = solver.Phase(self.intvar_lists["opt_time_goal"],
        #                      solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MAX_VALUE)

        # db_2 = solver.Phase([self.intervalvar_lists["make_span_interval"]], solver.INTERVAL_DEFAULT)

        # db_3 = solver.Phase([self.intervalvar_lists["make_span_interval"]] + interval_vars, solver.INTERVAL_DEFAULT)
        db_3 = solver.Phase(interval_vars, solver.INTERVAL_DEFAULT)

        resource_intervals = []
        for intervals in self.resource_intervals:
            resource_intervals += intervals

        intervalvars += resource_intervals

        if have_visit_ct:
            var_list = []
            for l in visit_intervals:
                for var in l:
                    var_list.append(var)
            db_visit_intervals = solver.Phase(var_list + resource_intervals, solver.INTERVAL_DEFAULT)

        db_4a = solver.Phase(self.intervalvar_lists["make_span_interval"], solver.INTERVAL_DEFAULT)

        intervalvars += self.intervalvar_lists["make_span_interval"]

        db_4 = solver.Phase(self.intvar_lists["opt_time_goal"],
                            solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
        # self.db = solver.Compose([db_0a, db_1, db_3])
        # self.db = solver.Compose([db_1, db_3])
        # self.db = solver.Compose([db_1, db_1b, db_3])

        # all interval variables db
        db_all_intervals = solver.Phase(intervalvars, solver.INTERVAL_DEFAULT)

        # we estimate the horizon when after we know how many configurations are needed
        horizons = []
        for key in self.roadmaps.keys():
            horizons += self.intvar_lists["horizon_" + key]
        db_horizon = solver.Phase(horizons, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)

        if have_visit_ct:
            decision_builders.append(db_visit_confs)
        if have_visiting_ct_var:
            pass
            # decision_builders.append(db_visiting_ct_var) # we should not search over the OVC variables

        # adding the horizon
        decision_builders.append(db_horizon)
        decision_builders.append(db_1)  # configurations

        ####################
        # limit = solver.FailuresLimit(20)
        limit = solver.TimeLimit(2000)
        # limit = solver.BranchesLimit(100)
        db_interval_once = solver.SolveOnce(db_all_intervals, [limit])
        decision_builders.append(db_interval_once)
        # decision_builders.append(db_all_intervals)
        ####################
        # TODO: comment in ?
        # decision_builders.append(db_3)  # interval vars
        # if have_visit_ct:
        #     decision_builders.append(db_visit_intervals)
        # if have_visiting_ct_var:
        #     pass
        #     # decision_builders.append(db_visiting_intervals_ct_var) # TODO: we should not search over the OVC intervals
        # decision_builders.append(db_4a)  # make span interval

        decision_builders.append(db_4)  # opt time goal

        # decision_builders.append(db_horizon)

        self.db = solver.Compose(decision_builders)
        # self.db = solver.Compose([db_1, db_3, db_4a, db_4])

        return

    def define_search_monitors(self):
        optimize = self.solver.Minimize(self.intvar_lists["opt_time_goal"][0], 1)
        search_log = self.solver.SearchLog(10000, self.intvar_lists["opt_time_goal"][0])

        self.search_monitors = [optimize, search_log]

    def print_model(self):
        pmv = self.solver.PrintModelVisitor()
        psv = self.solver.StatisticsModelVisitor()

        print pmv
        # self.solver.Accept(pmv)
        self.solver.Accept(psv)
        return

    def get_last_interval_vars(self):
        last_intervals = []
        for group in self.roadmaps.keys():
            # conf_vars = self.intvar_lists["c_" + group]
            interval_vars = self.intervalvar_lists["i_" + group]
            last_intervals.append(interval_vars[-1])
        return last_intervals

    def get_vars_for_group(self, group):
        conf_vars = self.intvar_lists["c_" + group]
        interval_vars = self.intervalvar_lists["i_" + group]

        return conf_vars, interval_vars

    def get_object_name(self, node_alias=None):
        return self.sosm.get_name_for_alias(node_alias)

    def constrain_conf_vars(self):
        for gn, rm in self.roadmaps.items():
            conf_domain = [int(v) for v in rm.get_nodes_of_major_component()]
            for conf_var in self.intvar_lists["c_" + gn]:  # type: pywrapcp.IntVar
                conf_var.SetValues(conf_domain)

    def constrain_conn_group(self):
        conn_var_dict = {}
        group_var_dict = {}
        for var in self.ordered_visiting_cst_vars:  # type: OrderedVisitingConstraintVar
            execution_group, conf_var, loc_var, connect_var, intervals = var.get_var_list()
            conn_var_dict[var.get_name()] = connect_var
            group_var_dict[var.get_name()] = execution_group

        ct = ConnectSameGroupCt(self.solver, conn_var_dict, group_var_dict)
        self.solver.Add(ct)

    def ovc_search(self, exp=None, luby_constant=5, EXECUTE_MOTION=False, time_limit=5000, seed=None, bc_solution=None,
                   bc_solver_stats=None, bc_extra_sol_files=None, bc_motion_req=None):
        self.constrain_conf_vars()
        self.constrain_conn_group()
        solver = self.solver
        rand = random.Random()
        if not seed:
            seed = rand.randint(1, 10000)

        solver.ReSeed(seed)


        # collect the variables and build the decision builders
        interval_list = []
        e_groups = []
        conf_vars = []
        connect_vars = []
        loc_vars = []

        # collect OVC variables
        for var in self.ordered_visiting_cst_vars:
            execution_group, conf_var, loc_var, connect_var, intervals = var.get_var_list()
            e_groups += [execution_group]
            conf_vars += conf_var
            loc_vars += loc_var
            connect_vars += connect_var
            interval_list += intervals

        # define dbs for OVC variables - VARIABLE AND VALUE SELECTION HEURISTICS!!!
        # visiting_intvar_db = self.solver.Phase(visiting_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
        execution_group_db = self.solver.Phase(e_groups, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_RANDOM_VALUE)
        conf_vars_db = self.solver.Phase(conf_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_RANDOM_VALUE)
        loc_vars_db = self.solver.Phase(loc_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_RANDOM_VALUE)
        # connect_vars_db = self.solver.Phase(connect_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
        connect_vars_db = self.solver.Phase(connect_vars, solver.CHOOSE_RANDOM, solver.ASSIGN_MIN_VALUE)
        # connect_vars_db = self.solver.Phase(connect_vars, solver.CHOOSE_RANDOM, solver.ASSIGN_CENTER_VALUE)

        # concat the OVC vars with a relevant order - VARIABLE ORDERING
        # visiting_intvar_db = solver.Compose([execution_group_db, conf_vars_db, connect_vars_db])
        visiting_intvar_db = solver.Compose([loc_vars_db, execution_group_db, conf_vars_db, connect_vars_db])
        # visiting_intvar_db = solver.Compose([connect_vars_db, conf_vars_db, execution_group_db])


        visiting_interval_db = self.solver.Phase(interval_list, solver.INTERVAL_DEFAULT)

        visiting_vars = []
        for var in self.ordered_visiting_cst_vars:
            # int_var, intervals = var.get_var_list()
            visiting_vars += var._conf_values
            visiting_vars += [var._execution_group]
            # visiting_vars += var._wt_build

        visiting_conf_db = self.solver.Phase(visiting_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_RANDOM_VALUE)


        # visiting_intvar_db contains OVC dbs and self.db contains motion model variables
        # complete_db = self.solver.Compose([visiting_intvar_db, self.db, visiting_interval_db])
        complete_db = self.solver.Compose([visiting_intvar_db, self.db])  # we should not search over OVC intervals
        # complete_db = self.solver.Compose([self.db, visiting_intvar_db])  # we should not search over OVC intervals

        # define the restart strategy
        luby_restart = solver.LubyRestart(luby_constant)
        const_restart = solver.ConstantRestart(500)

        final_db = solver.Compose([complete_db])
        # final_db = solver.SolveOnce(final_db, [])

        # debug search tracer
        trace = solver.SearchTrace('debug')

        # define search
        # self.solver.NewSearch(final_db, self.search_monitors + [self.solver.TimeLimit(time_limit)])
        self.solver.NewSearch(final_db, self.search_monitors + [self.solver.TimeLimit(time_limit), luby_restart])
        # self.solver.NewSearch(final_db, self.search_monitors + [self.solver.TimeLimit(time_limit), const_restart])

        # solution variables
        count = 0

        solutions = []
        solution_end_times = []
        start_time = datetime.now()
        elapsed_time = []

        new_opt_bound = np.Inf

        # start solver
        while self.solver.NextSolution():
            elapsed_time.append((datetime.now() - start_time).total_seconds())
            exp.add_time_to_solution(elapsed_time[-1])
            count += 1
            print "Solution: {}".format(count)

            if bc_solver_stats:
                bc_solver_stats(SolverStats(solver))
            # print solver._Solver__python_constraints[2]
            solution = self.print_solution()
            # send solution back
            if bc_solution:
                bc_solution(solution)
            # bc(solution)
            # gantt = self.save_solution_gantt()
            # if bc_extra_sol_files:
            #     bc_extra_sol_files(gantt)

            last_intervals = self.get_last_interval_vars()
            earliest_ends = [last_intervals[k].EndMin() for k in range(len(last_intervals))]

            # if np.max([i_right[-1].EndMin(), i_left[-1].EndMin()]) < new_opt_bound:
            if np.max(earliest_ends) < new_opt_bound:
                new_opt_bound = np.max(earliest_ends)

                solution_end_times.append(new_opt_bound)
                exp.add_solution_quality(new_opt_bound)

                if bc_motion_req:
                    motion_requests = self.build_ros_trajectory_request()
                    bc_motion_req(motion_requests)

                solutions.append(motion_requests)

                if EXECUTE_MOTION:
                    self.send_motion_requests(motion_requests)

            start_time = datetime.now()

        self.solver.EndSearch()

        if len(solutions) == 0:
            total_search_time = (datetime.now() - start_time).total_seconds()
        else:
            total_search_time = np.sum(elapsed_time) + (datetime.now() - start_time).total_seconds()

        if count > 0:
            if EXECUTE_MOTION:
                self.trigger_build_plan()

        return elapsed_time, solutions, solution_end_times, total_search_time

    def test_solution(self, solution):
        if solution.Value(self.allocation_vars["wt_arm_assignment_vars"][0]) < -5:
            return False

        return True

    def make_neighbor(self, solution_old, k):

        # copy assignment
        assert k > 0
        solution = self.solver.Assignment(solution_old)

        rand = np.random

        for n_op in range(k):

            decision = rand.randint(0, 100)

            if decision < 30:
                # flip arm assignment
                flip_index = rand.randint(0, 5)
                arm_val = solution.Value(self.allocation_vars["wt_arm_assignment_vars"][flip_index])
                if arm_val == 1:
                    new_val = 2
                else:
                    new_val = 1
                solution.SetValue(self.allocation_vars["wt_arm_assignment_vars"][flip_index], new_val)
                # return solution

            if 30 <= decision < 60:
                # switch part assignment
                flip_index_1 = rand.randint(0, 5)
                flip_index_2 = rand.randint(0, 5)
                while (flip_index_1 == flip_index_2):
                    flip_index_2 = rand.randint(0, 5)

                part_1 = solution.Value(self.allocation_vars["wt_brush_assignment_vars"][flip_index_1])
                part_2 = solution.Value(self.allocation_vars["wt_brush_assignment_vars"][flip_index_2])

                solution.SetValue(self.allocation_vars["wt_brush_assignment_vars"][flip_index_1], part_2)
                solution.SetValue(self.allocation_vars["wt_brush_assignment_vars"][flip_index_2], part_1)
                # return solution

            if 60 <= decision < 100:
                # switch manufacturing order
                flip_index_1 = rand.randint(0, 5)
                flip_index_2 = rand.randint(0, 5)
                while (flip_index_1 == flip_index_2):
                    flip_index_2 = rand.randint(0, 5)

                part_1 = solution.Value(self.allocation_vars["wt_build_order_vars"][flip_index_1])
                part_2 = solution.Value(self.allocation_vars["wt_build_order_vars"][flip_index_2])

                solution.SetValue(self.allocation_vars["wt_build_order_vars"][flip_index_1], part_2)
                solution.SetValue(self.allocation_vars["wt_build_order_vars"][flip_index_2], part_1)
                # return solution

        return solution

    def local_search_cost_function(self, solution):
        sosm = self.sosm

        # WT from left to right for left arm: 3,4,5,9,x
        # WT from left to right for right arm: 3,4,5,9,8
        # Brushes from left to right for right arm: 6,7,2,1,0
        # Brushes from left to right for left arm: 7,8,2,1,0

        wt_poses = {"right_arm": {1: 3, 2: 3, 3: 5, 4: 9, 5: 8},
                    "left_arm": {1: 3, 2: 3, 3: 5, 4: 9}}
        brush_poses = {"right_arm": {1: 6, 2: 7, 3: 2, 4: 1, 5: 0},
                       "left_arm": {1: 7, 2: 8, 3: 2, 4: 1, 5: 0}}
        tool_poses = {"right_arm": {2: 10},
                      "left_arm": {1: 6}}

        # available_wts = [1, 2, 3, 4, 5]
        available_wts = {"WT_1": 1, "WT_2": 2, "WT_3": 3, "WT_4": 4, "WT_5": 5}

        # available_tools = [1, 2]
        available_tools = {"Tool_1": 1, "Tool_2": 2}
        # available_brushes = [1, 2, 3, 4, 5]
        available_brushes = {"Brush_1": 1, "Brush_2": 2, "Brush_3": 3, "Brush_4": 4, "Brush_5": 5}
        available_groups = {1: "right_arm", 2: "left_arm"}

        wt_poses = sosm.get_alias_to_poses_for_type("plate")

        brush_poses = sosm.get_alias_to_poses_for_type("brush")

        tool_poses = sosm.get_alias_to_poses_for_type("torpedo")

        available_wts = sosm.get_available_objects_of_type("plate")

        available_tools = sosm.get_available_objects_of_type("torpedo")

        available_brushes = sosm.get_available_objects_of_type("brush")

        available_groups = {"right_arm": 2, "left_arm": 1}

        alloc = self.allocation_vars
        alloc_dicts = self.alloc_dict
        # visiting_vars = self.ordered_visiting_cst_vars  # type: list[OrderedVisitingConstraintVar]
        cost_groups = {}

        # for index,
        paths = {}
        for g in available_groups.keys():
            paths[g] = [tool_poses[g].values()[0]]

        for wt in alloc["wt_build_order_vars"]:
            wt_val = solution.Value(wt)
            execution_group_val = solution.Value(alloc_dicts["wt_arm_assignment_vars"][wt_val])
            execution_group = available_groups.keys()[available_groups.values().index(execution_group_val)]
            brush_val = solution.Value(alloc_dicts["wt_brush_assignment_vars"][wt_val])
            try:
                wt_pose = wt_poses[execution_group][wt_val]
                brush_pose = brush_poses[execution_group][brush_val]
            except KeyError:
                return np.Inf

            paths[execution_group].append(wt_pose)
            paths[execution_group].append(brush_pose)
            paths[execution_group].append(wt_pose)

        print "still here"

        cost_conflict = 0
        for gn, path in paths.items():

            dist = self.roadmaps[gn].calc_path_distance(path)
            group_tuple = (gn, list(self.clash.groups.difference([gn]))[0])
            conflicts = np.array([len(self.clash.clashes[group_tuple][loc]) for loc in path])
            average_conflict = 1.0 * sum(conflicts * conflicts) / (len(conflicts) * 120 * 120)
            try:
                cost_groups[gn] += dist
            except KeyError:
                cost_groups[gn] = dist

            cost_conflict += 100 / average_conflict

        # conf_WT_0_0 (5) | conf_WT_0_1 (3) | conf_WT_0_2 (2) | conf_WT_0_3 (3) | execution_groupWT_1 (1)
        return max(cost_groups.values()) + cost_conflict

    def print_solution(self):
        solution_dict = {}

        for k, v in self.intvar_lists.items():
            print k
            print v
            for value in v:
                variable = IntVariable(value)
                solution_dict[variable.name] = variable

        for k, v in self.intervalvar_lists.items():
            print k
            print v
            for value in v:
                ivar = IntervalVariable(value)
                solution_dict[value.Name()] = ivar



                # def StartExpr(self):
                #     return _pywrapcp.IntervalVar_StartExpr(self)
                #
                # def DurationExpr(self):
                #     return _pywrapcp.IntervalVar_DurationExpr(self)
                #
                # def EndExpr(self):
                #     return _py

        for k, v in self.resource_intervals.items():
            print k
            print v
            for value in v:
                ivar = IntervalVariable(value)
                solution_dict[value.Name()] = ivar


        try:
            for var in self.ordered_visiting_cst_vars:
                ovc = OrderedVisitingConstraint(var)
                solution_dict[ovc.name] = ovc
                for v in var.get_var_list():
                    print v
        except AttributeError:
            pass

        # self.save_solution_gantt()
        return solution_dict

    def save_solution_gantt(self):
        df = []

        for group in self.roadmaps.keys():
            conf_vars, interval_vars = self.get_vars_for_group(group)
            for index, var in enumerate(interval_vars):
                start = var.StartExpr()
                start_in_seconds = str(start)
                start_m, start_s = divmod(int(start_in_seconds), 60)
                start_h, start_m = divmod(start_m, 60)

                finish = var.EndExpr()
                finish_in_seconds = str(finish)
                finish_m, finish_s = divmod(int(finish_in_seconds), 60)
                finish_h, finish_m = divmod(finish_m, 60)
                # finish =
                s_str = "2017-01-01 {:02}:{:02}:{:02}".format(start_h, start_m, start_s)
                f_str = "2017-01-01 {:02}:{:02}:{:02}".format(finish_h, finish_m, finish_s)
                if finish_in_seconds == start_in_seconds:
                    continue

                if index % 2 == 1:
                    resource = "Waiting"
                    descr = "Waiting at node {}".format(conf_vars[index / 2].Value())
                else:
                    resource = "Travelling"
                    descr = "Travelling from node {} to {}".format(conf_vars[(index - 1) / 2].Value(),
                                                                   conf_vars[(index + 1) / 2].Value())

                df.append(dict(Task=group, Start="2017-01-01 {:02}:{:02}:{:02}".format(start_h, start_m, start_s),
                               Finish="2017-01-01 {:02}:{:02}:{:02}".format(finish_h, finish_m, finish_s),
                               Resource=resource,
                               Description=descr))

        for visit_ct in self.ordered_visiting_cst_vars:
            var = visit_ct._spanning_visit_interval
            start = var.StartExpr()
            start_in_seconds = str(start)
            start_m, start_s = divmod(int(start_in_seconds), 60)
            start_h, start_m = divmod(start_m, 60)

            finish = var.EndExpr()
            finish_in_seconds = str(finish)
            finish_m, finish_s = divmod(int(finish_in_seconds), 60)
            finish_h, finish_m = divmod(finish_m, 60)
            # finish =
            s_str = "2017-01-01 {:02}:{:02}:{:02}".format(start_h, start_m, start_s)
            f_str = "2017-01-01 {:02}:{:02}:{:02}".format(finish_h, finish_m, finish_s)

            if visit_ct._execution_group.Value() == 1:
                resource = "left_arm_gripper"
            if visit_ct._execution_group.Value() == 2:
                resource = "right_arm_gripper"
            # resource = str(visit_ct._execution_group.Value())
            descr = str(var.__repr__())

            df.append(dict(Task=resource, Start="2017-01-01 {:02}:{:02}:{:02}".format(start_h, start_m, start_s),
                           Finish="2017-01-01 {:02}:{:02}:{:02}".format(finish_h, finish_m, finish_s),
                           Resource=resource,
                           Description=descr))

        for key, var_list in self.resource_intervals.items():
            for var in var_list:
                start = var.StartExpr()
                start_in_seconds = str(start)
                start_m, start_s = divmod(int(start_in_seconds), 60)
                start_h, start_m = divmod(start_m, 60)

                finish = var.EndExpr()
                finish_in_seconds = str(finish)
                finish_m, finish_s = divmod(int(finish_in_seconds), 60)
                finish_h, finish_m = divmod(finish_m, 60)
                # finish =
                s_str = "2017-01-01 {:02}:{:02}:{:02}".format(start_h, start_m, start_s)
                f_str = "2017-01-01 {:02}:{:02}:{:02}".format(finish_h, finish_m, finish_s)

                resource = key
                descr = str(var.__repr__())

                df.append(dict(Task=key, Start="2017-01-01 {:02}:{:02}:{:02}".format(start_h, start_m, start_s),
                               Finish="2017-01-01 {:02}:{:02}:{:02}".format(finish_h, finish_m, finish_s),
                               Resource=resource,
                               Description=descr))

        colors = dict(Travelling='rgb(46, 137, 205)',
                      Waiting='rgb(114, 44, 121)',
                      right_arm_gripper='rgb(198, 47, 105)',
                      left_arm_gripper='rgb(58, 149, 136)',
                      Rest='rgb(107, 127, 135)')

        fig = ff.create_gantt(df, colors=colors, index_col='Resource', title='Manipulation Plan',
                              show_colorbar=True, showgrid_x=True, showgrid_y=True, group_tasks=True)
        # py.iplot(fig, filename="gantt_solution", world_readable=True)

        plot = py.plot(fig, filename="gantt_solution_{}.html".format(datetime.now()), auto_open=False)

        return plot

    def load_roadmap(self, file_name, group_name="right_arm"):
        """

        :param file_name: file name of roadmap file
        :param group_name: deprecated - has no effect and will be removed
        """
        # if group_name in self.roadmaps.keys():
        #     del self.roadmaps[group_name]
        #     rospy.loginfo("Deleted RoadMap for {} to load another one for same group.".format(group_name))
        roadmapfactory = RoadMapFactory()
        rm = roadmapfactory.load_from_file(file_name)
        self.roadmaps[rm.get_group_name()] = rm
        # self.roadmaps[group_name] = RoadMap(file_name, group_name)
        rospy.loginfo("Loaded RoadMap for {}.".format(group_name))

    def check_planning_requirements(self):
        # for key in self.roadmaps.keys():
        #     assert key in self.clash.keys()
        self.clash.verify_clash_roadmap_combinations(self.roadmaps)

    def trigger_build_plan(self):
        TRIGGER_ROS_SERVICE = True
        if TRIGGER_ROS_SERVICE:
            self.trigger_build_plan_client(EmptyRequest())

    def build_ros_trajectory_request(self):
        TRIGGER_ROS_SERVICE = True
        requests = []

        if TRIGGER_ROS_SERVICE:
            try:
                pass
                # rospy.init_node("CSP_MOTION_SOLVER")
            except rospy.exceptions.ROSException:
                pass

            for group, rm in self.roadmaps.items():
                conf_vars, interval_vars = self.get_vars_for_group(group)

                req = BuildMotionAddTrajectoryRequest()

                import cPickle as pickle
                req.prm_pickle = pickle.dumps(rm, pickle.HIGHEST_PROTOCOL)

                req.move_group = group
                req.prm_name = self.roadmaps[group].get_fingerprint()
                index = 0
                for interval in interval_vars:
                    req.time.append(interval.StartMin())
                    req.time.append(interval.EndMin())

                    if np.mod(index, 2) == 0:
                        # interval of point
                        req.prm_pos.append(conf_vars[index / 2].Value())
                        req.prm_pos.append(conf_vars[index / 2].Value())
                    elif np.mod(index, 2) == 1:
                        # interval of travel
                        req.prm_pos.append(conf_vars[(index - 1) / 2].Value())
                        req.prm_pos.append(conf_vars[(index + 1) / 2].Value())

                    index += 1

                print req.move_group
                print(req.time)
                print(req.prm_pos)
                requests.append(req)
            return requests

    def send_motion_requests(self, requests):
        for req in requests:
            self.send_trajectory_data_client(req)

    def trigger_build_plan_client(self, request):
        # type: (EmptyRequest)->EmptyResponse
        rospy.wait_for_service('MOTION_DISPATCHER/BUILD_MOTION_PLAN')
        try:
            build_plan_srv = rospy.ServiceProxy('MOTION_DISPATCHER/BUILD_MOTION_PLAN', Empty)
            req = request
            res = build_plan_srv(req)  # type: EmptyResponse
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def send_trajectory_data_client(self, request):
        rospy.wait_for_service('MOTION_DISPATCHER/ADD_TRAJECTORY')
        try:
            send_trajectory_srv = rospy.ServiceProxy('MOTION_DISPATCHER/ADD_TRAJECTORY', BuildMotionAddTrajectory)
            req = request  # type: BuildMotionAddTrajectoryRequest
            res = send_trajectory_srv(req)  # type: BuildMotionAddTrajectoryResponse
            print res.msg
            print res.success
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
