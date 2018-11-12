#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from ortools.constraint_solver import pywrapcp
import itertools


class IntervalClashPathCt(pywrapcp.PyConstraint):
    def __init__(self, solver, c_left=[], c_right=[], i_left=[], i_right=[], rm1=None, rm2=None, clash={}):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._clash = clash
        self._c_self = c_left
        self._c_other = c_right
        self._i_self = i_left
        self._i_other = i_right
        self._rm_self = rm1
        self._rm_other = rm2
        # self._spacial_overlap = spacial_overlap
        self._demons = []

        # print 'Constraint built'
        # print self._xl

    def Post(self):
        # print 'in Post()'
        # self._demon_r = DemonProp(self._xr, self)
        # self._demon = Demon(self.InitialPropagate)
        c_index = 0
        for x in self._c_self:
            self._demons.append(self.DelayedDemon(IntervalClashPathCt.Propagate, x, c_index))
            x.WhenBound(self._demons[-1])
            c_index += 1
            # self._xr.WhenBound(self._demon_r)
            # self._x1.WhenDomain(self._demon)
            # print 'out of Post()'

    def InitialPropagate(self):
        pass

    def Propagate(self, var, index):
        solver = self.solver()
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
                path = self._rm_self.find_path_prm(start, val, True)
                val_dict[-1] = val_dict[-1].union(set(path))
            if self._c_self[index + 1].Bound():
                next_val_bound = True
                goal = self._c_self[index + 1].Value()
                path = self._rm_self.find_path_prm(val, goal, True)
                val_dict[1] = val_dict[1].union(set(path))
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
                if interval not in affected_intervals_path[-1]:
                    # check if it should be
                    occupied_configurations = set()
                    lower_conf_idx = (k - 1) / 2
                    upper_conf_idx = (k + 1) / 2
                    if self._c_other[lower_conf_idx].Bound():
                        occupied_configurations.add(self._c_other[lower_conf_idx].Value())
                    if self._c_other[upper_conf_idx].Bound():
                        occupied_configurations.add(self._c_other[upper_conf_idx].Value())
                    if self._c_other[lower_conf_idx].Bound() and self._c_other[upper_conf_idx].Bound():
                        start = self._c_other[lower_conf_idx].Value()
                        target = self._c_other[upper_conf_idx].Value()
                        path_other = self._rm_other.find_path_prm(start, target, True)
                        occupied_configurations = occupied_configurations.union(set(path_other))

                    if len(occupied_configurations.intersection(non_colliding_values[-1])) != len(
                            occupied_configurations):
                        # some values in occupied_configurations are not in non_colliding_values and therefor we have to add this interval
                        affected_intervals_path[-1].add(interval)
                if interval not in affected_intervals_path[0]:
                    # check if it should be
                    occupied_configurations = set()
                    lower_conf_idx = (k - 1) / 2
                    upper_conf_idx = (k + 1) / 2
                    if self._c_other[lower_conf_idx].Bound():
                        occupied_configurations.add(self._c_other[lower_conf_idx].Value())
                    if self._c_other[upper_conf_idx].Bound():
                        occupied_configurations.add(self._c_other[upper_conf_idx].Value())
                    if self._c_other[lower_conf_idx].Bound() and self._c_other[upper_conf_idx].Bound():
                        start = self._c_other[lower_conf_idx].Value()
                        target = self._c_other[upper_conf_idx].Value()
                        path_other = self._rm_other.find_path_prm(start, target, True)
                        occupied_configurations = occupied_configurations.union(set(path_other))

                    if len(occupied_configurations.intersection(non_colliding_values[0])) != len(
                            occupied_configurations):
                        # some values in occupied_configurations are not in non_colliding_values and therefor we have to add this interval
                        affected_intervals_path[0].add(interval)
                if interval not in affected_intervals_path[1]:
                    # check if it should be
                    occupied_configurations = set()
                    lower_conf_idx = (k - 1) / 2
                    upper_conf_idx = (k + 1) / 2
                    if self._c_other[lower_conf_idx].Bound():
                        occupied_configurations.add(self._c_other[lower_conf_idx].Value())
                    if self._c_other[upper_conf_idx].Bound():
                        occupied_configurations.add(self._c_other[upper_conf_idx].Value())
                    if self._c_other[lower_conf_idx].Bound() and self._c_other[upper_conf_idx].Bound():
                        start = self._c_other[lower_conf_idx].Value()
                        target = self._c_other[upper_conf_idx].Value()
                        path_other = self._rm_other.find_path_prm(start, target, True)
                        occupied_configurations = occupied_configurations.union(set(path_other))

                    if len(occupied_configurations.intersection(non_colliding_values[1])) != len(
                            occupied_configurations):
                        # some values in occupied_configurations are not in non_colliding_values and therefor we have to add this interval
                        affected_intervals_path[1].add(interval)

        for i in own_intervals_dict.keys():
            it = itertools.product(own_intervals_dict[i], affected_intervals_path[i])
            for intervals in it:
                ct = solver.TemporalDisjunction(intervals[0], intervals[1])
                solver.AddConstraint(ct)
