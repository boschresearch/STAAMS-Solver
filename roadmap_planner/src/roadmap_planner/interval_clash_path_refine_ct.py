#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import math
from ortools.constraint_solver import pywrapcp
import itertools
from CustomConstraints import VarContainerCt


class IntervalClashPathRefineCt(pywrapcp.PyConstraint):
    # TODO: get scaling factor from central point
    SCALING_FACTOR = 10

    def __init__(self, solver, c_left=[], c_right=[], i_left=[], i_right=[], rm1=None, rm2=None, clash={}, roadmap_planner=None):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._rp = roadmap_planner
        self._clash = clash
        self._c_self = c_left
        self._c_other = c_right
        self._i_self = i_left
        self._i_other = i_right
        self._rm_self = rm1
        self._rm_other = rm2
        # self._spacial_overlap = spacial_overlap
        self._demons = []

        self._created_intervals = {}

        # print 'Constraint built'
        # print self._xl

    def Post(self):
        # print 'in Post()'
        # self._demon_r = DemonProp(self._xr, self)
        # self._demon = Demon(self.InitialPropagate)
        c_index = 0
        for x in self._c_self:
            self._demons.append(self.DelayedDemon(IntervalClashPathRefineCt.Propagate, x, c_index))
            x.WhenBound(self._demons[-1])
            c_index += 1
            # self._xr.WhenBound(self._demon_r)
            # self._x1.WhenDomain(self._demon)
            # print 'out of Post()'

    def InitialPropagate(self):
        self._created_intervals = {}

    def Propagate(self, var, index):
        LOG_VARS = True
        solver = self.solver()  # type: pywrapcp.Solver
        val = var.Value()
        # index = self._c_self.index(var)
        val_dict = {}
        val_dict[-1] = set([val])
        val_dict[0] = set([val])
        val_dict[1] = set([val])
        non_colliding_values = {}  # type: dict[int, set]
        prev_val_bound = False
        next_val_bound = False

        try:
            if self._c_self[index - 1].Bound():
                prev_val_bound = True
                start = self._c_self[index - 1].Value()
                path_0 = self._rm_self.find_path_prm(start, val, True)
                val_dict[-1] = val_dict[-1].union(set(path_0))
            if self._c_self[index + 1].Bound():
                next_val_bound = True
                goal = self._c_self[index + 1].Value()
                path_1 = self._rm_self.find_path_prm(val, goal, True)
                val_dict[1] = val_dict[1].union(set(path_1))
        except IndexError:
            pass

        non_colliding_values[0] = set(self._clash[val])
        non_colliding_values[-1] = set(self._clash[val])
        non_colliding_values[1] = set(self._clash[val])
        if prev_val_bound:
            for value in val_dict[-1]:
                non_colliding_values[-1] = non_colliding_values[-1].intersection(self._clash[value])
        if next_val_bound:
            for value in val_dict[1]:
                non_colliding_values[1] = non_colliding_values[1].intersection(self._clash[value])

        # collect the indexes of the onw intervals using the newly bound configuration
        own_intervals = [2 * index - 1, 2 * index, 2 * index + 1]
        own_intervals_dict = {}
        s = -1
        for i in own_intervals:
            try:
                if i < 0:
                    raise IndexError
                own_intervals_dict[s] = [self._i_self[i]]
            except IndexError:
                pass
            s += 1
            # i += 1

        # own_intervals = [2 * index - 2, 2 * index - 1, 2 * index, 2 * index + 1, 2 * index + 2]
        # own_intervals = [np.min([np.max([2 * index + i, 0]), len(self._i_self)-1]) for i in range(-2, 3, 1)]
        # min = np.min(own_intervals)
        # max = np.max(own_intervals)
        if index == 0:
            own_intervals.pop(0)
        elif own_intervals[-1] >= len(self._i_self):
            own_intervals.pop(-1)

        # own_intervals = list(set(own_intervals))

        # checking which intervals are affected
        affected_intervals_path = {}  # type: dict[int, set]
        affected_intervals_path[-1] = set()
        affected_intervals_path[0] = set()
        affected_intervals_path[1] = set()

        for k, interval in enumerate(self._i_other):
            # size of intersection between path and allowed values == path
            if k % 2 == 0:
                # we have a waiting interval
                if self._c_other[k / 2].Bound():
                    if self._c_other[k / 2].Value() not in non_colliding_values[0]:
                        affected_intervals_path[0] = affected_intervals_path[0].union(set([interval]))
                    if self._c_other[k / 2].Value() not in non_colliding_values[-1]:
                        affected_intervals_path[-1] = affected_intervals_path[-1].union(set([interval]))
                    if self._c_other[k / 2].Value() not in non_colliding_values[1]:
                        affected_intervals_path[1] = affected_intervals_path[1].union(set([interval]))
            else:
                # we have a travel interval
                USE_REFINEMENT = True
                if interval not in affected_intervals_path[-1]:
                    # check if it should be
                    if self.check_path_collision(k, non_colliding_values[-1]):
                        # path_0 and path_other are colliding
                        if USE_REFINEMENT and prev_val_bound:
                            path_self = path_0
                            path_other, path_other_valid = self.get_path_for_interval(k, 1)
                            for idx_self, confs in enumerate(zip(path_self[:-1], path_self[1:])):
                                conf_1, conf_2 = confs
                                for idx_other, confs in enumerate(zip(path_other[:-1], path_other[1:])):
                                    c1o, c2o = confs
                                    if self.check_path_segments_for_collision(conf_1, conf_2, c1o, c2o):
                                        # create intervals for the colliding path segments
                                        # i_s = self._rp.get_refine_interval("refine_interval_{}_{}".format(
                                        #                              self._i_self[index].Name(),
                                        #                              idx_self))
                                        i_s = solver.IntervalVar(0, 10000, 0, 10000, 0, 10000, False,
                                                                 "refine_interval_{}_{}".format(
                                                                     self._i_self[index].Name(),
                                                                     idx_self))  # type: pywrapcp.IntervalVar
                                        i_o = solver.IntervalVar(0, 10000, 0, 10000, 0, 10000, False,
                                                                 "refine_interval_{}_{}".format(self._i_other[k].Name(),
                                                                                                idx_other))  # type: pywrapcp.IntervalVar

                                        self._created_intervals[i_s.Name()] = i_s
                                        self._created_intervals[i_o.Name()] = i_o
                                        start_floor = math.floor(IntervalClashPathRefineCt.SCALING_FACTOR *
                                            self._rm_self.calc_path_distance(path_self[:idx_self + 1]))
                                        end_ceil = math.ceil(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                             self._rm_self.calc_path_distance(path_self[:idx_self + 2]))

                                        ct_1 = i_s.StartsAtStartWithDelay(self._i_self[index * 2], int(start_floor))
                                        ct_2 = i_s.EndsAtStartWithDelay(self._i_self[index * 2], int(end_ceil))

                                        solver.AddConstraint(ct_1)
                                        solver.AddConstraint(ct_2)

                                        start_floor = math.floor(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                                 self._rm_other.calc_path_distance(path_other[:idx_other + 1]))
                                        end_ceil = math.ceil(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                             self._rm_other.calc_path_distance(path_other[:idx_other + 2]))

                                        ct_1 = i_o.StartsAtStartWithDelay(self._i_other[k], int(start_floor))
                                        ct_2 = i_o.EndsAtStartWithDelay(self._i_other[k], int(end_ceil))

                                        solver.AddConstraint(ct_1)
                                        solver.AddConstraint(ct_2)

                                        ct = solver.TemporalDisjunction(i_s, i_o)
                                        solver.AddConstraint(ct)

                        else:
                            affected_intervals_path[-1].add(interval)

                if interval not in affected_intervals_path[0]:
                    # check if it should be
                    if self.check_path_collision(k, non_colliding_values[0]):
                        if USE_REFINEMENT:
                            path_other, path_other_valid = self.get_path_for_interval(k, 1)
                            conf_self = self._c_self[index]
                            for idx_other, confs in enumerate(zip(path_other[:-1], path_other[1:])):
                                c1o, c2o = confs
                                if self.check_path_segments_for_collision(conf_self.Value(), conf_self.Value(), c1o,
                                                                          c2o):
                                    # create intervals for the colliding path segments
                                    i_s = self._i_self[index * 2]  # type: pywrapcp.IntervalVar
                                    i_o = solver.IntervalVar(0, 10000, 0, 10000, 0, 10000, False,
                                                             "refine_interval_{}_{}".format(
                                                                 self._i_other[k].Name(),
                                                                 idx_other))  # type: pywrapcp.IntervalVar

                                    # self._created_intervals[i_s.Name()] = i_s
                                    self._created_intervals[i_o.Name()] = i_o

                                    start_floor = math.floor(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                             self._rm_other.calc_path_distance(path_other[:idx_other + 1]))
                                    end_ceil = math.ceil(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                         self._rm_other.calc_path_distance(path_other[:idx_other + 2]))

                                    ct_1 = i_o.StartsAtStartWithDelay(self._i_other[k], int(start_floor))
                                    ct_2 = i_o.EndsAtStartWithDelay(self._i_other[k], int(end_ceil))

                                    solver.AddConstraint(ct_1)
                                    solver.AddConstraint(ct_2)

                                    ct = solver.TemporalDisjunction(i_s, i_o)
                                    solver.AddConstraint(ct)
                        else:
                            affected_intervals_path[0].add(interval)

                if interval not in affected_intervals_path[1]:
                    # check if it should be
                    if self.check_path_collision(k, non_colliding_values[1]):
                        if USE_REFINEMENT and next_val_bound:
                            path_self = path_1
                            path_other, path_other_valid = self.get_path_for_interval(k, 1)
                            for idx_self, confs in enumerate(zip(path_self[:-1], path_self[1:])):
                                conf_1, conf_2 = confs
                                for idx_other, confs in enumerate(zip(path_other[:-1], path_other[1:])):
                                    c1o, c2o = confs
                                    if self.check_path_segments_for_collision(conf_1, conf_2, c1o, c2o):
                                        # create intervals for the colliding path segments
                                        i_s = solver.IntervalVar(0, 10000, 0, 10000, 0, 10000, False,
                                                                 "refine_interval_{}_{}".format(
                                                                     self._i_self[index].Name(),
                                                                     idx_self))  # type: pywrapcp.IntervalVar
                                        i_o = solver.IntervalVar(0, 10000, 0, 10000, 0, 10000, False,
                                                                 "refine_interval_{}_{}".format(self._i_other[k].Name(),
                                                                                                idx_other))  # type: pywrapcp.IntervalVar

                                        self._created_intervals[i_s.Name()] = i_s
                                        self._created_intervals[i_o.Name()] = i_o

                                        start_floor = math.floor(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                                 self._rm_self.calc_path_distance(path_self[:idx_self + 1]))
                                        end_ceil = math.ceil(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                             self._rm_self.calc_path_distance(path_self[:idx_self + 2]))

                                        ct_1 = i_s.StartsAtStartWithDelay(self._i_self[index * 2], int(start_floor))
                                        ct_2 = i_s.EndsAtStartWithDelay(self._i_self[index * 2], int(end_ceil))

                                        solver.AddConstraint(ct_1)
                                        solver.AddConstraint(ct_2)

                                        start_floor = math.floor(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                                 self._rm_other.calc_path_distance(path_other[:idx_other + 1]))
                                        end_ceil = math.ceil(IntervalClashPathRefineCt.SCALING_FACTOR *
                                                             self._rm_other.calc_path_distance(path_other[:idx_other + 2]))

                                        ct_1 = i_o.StartsAtStartWithDelay(self._i_other[k], int(
                                            start_floor))  # type: pywrapcp.PyConstraint
                                        ct_2 = i_o.EndsAtStartWithDelay(self._i_other[k], int(end_ceil))

                                        # print ct_1.DebugString()

                                        solver.AddConstraint(ct_1)
                                        solver.AddConstraint(ct_2)

                                        ct = solver.TemporalDisjunction(i_s, i_o)
                                        solver.AddConstraint(ct)
                        else:
                            affected_intervals_path[1].add(interval)

        for i in own_intervals_dict.keys():
            it = itertools.product(own_intervals_dict[i], affected_intervals_path[i])
            for intervals in it:
                ct = solver.TemporalDisjunction(intervals[0], intervals[1])
                solver.AddConstraint(ct)

    def check_path_segments_for_collision(self, c1s, c2s, c1o, c2o):
        non_col_set = self._clash[c1s].intersection(self._clash[c2s])
        if c1o not in non_col_set or c2o not in non_col_set:
            return True
        else:
            return False

    def get_path_for_interval(self, k, group=0):
        '''
        get the path, i.e. a list of configurations visited in an interval
        :param k: index of the interval
        :param group: 0 means i_self, 1 means i_other
        :return: get the path, i.e. a list of configurations visited in an interval
        second return if the path is valid
        '''
        lower_conf_idx = (k - 1) / 2
        upper_conf_idx = (k + 1) / 2
        if group == 0:
            c = self._c_self
        elif group == 1:
            c = self._c_other
        else:
            assert False
        if c[lower_conf_idx].Bound() and c[upper_conf_idx].Bound():
            start = c[lower_conf_idx].Value()
            target = c[upper_conf_idx].Value()
            path = self._rm_other.find_path_prm(start, target, True)
            return path, True
        else:
            return [], False

    def check_path_collision(self, k, non_colliding_values):
        occupied_configurations = set()
        lower_conf_idx = (k - 1) / 2
        upper_conf_idx = (k + 1) / 2

        lower_conf_bound = self._c_other[lower_conf_idx].Bound()
        upper_conf_bound = self._c_other[upper_conf_idx].Bound()

        if lower_conf_bound and upper_conf_bound:
            start = self._c_other[lower_conf_idx].Value()
            target = self._c_other[upper_conf_idx].Value()
            path_other = self._rm_other.find_path_prm(start, target, True)
            occupied_configurations = occupied_configurations.union(set(path_other))
        elif lower_conf_bound:
            occupied_configurations.add(self._c_other[lower_conf_idx].Value())
        elif upper_conf_bound:
            occupied_configurations.add(self._c_other[upper_conf_idx].Value())

        # if self._c_other[lower_conf_idx].Bound():
        #     occupied_configurations.add(self._c_other[lower_conf_idx].Value())
        # if self._c_other[upper_conf_idx].Bound():
        #     occupied_configurations.add(self._c_other[upper_conf_idx].Value())
        # if self._c_other[lower_conf_idx].Bound() and self._c_other[upper_conf_idx].Bound():
        #     start = self._c_other[lower_conf_idx].Value()
        #     target = self._c_other[upper_conf_idx].Value()
        #     path_other = self._rm_other.find_path_prm(start, target, True)
        #     occupied_configurations = occupied_configurations.union(set(path_other))

        if not occupied_configurations.issubset(non_colliding_values):
            # if len(occupied_configurations.intersection(non_colliding_values)) != len(
            #         occupied_configurations):
            return True
        else:
            return False
