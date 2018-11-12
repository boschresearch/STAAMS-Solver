#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from ortools.constraint_solver import pywrapcp

from roadmap_planner.loc_conf_ct import LocConfCt
from roadmap_planning_common_msgs.msg import Range


class OrderedVisitingConstraintVar:
    def __init__(self, solver, n_poses, time_max, steps_max, n_conf_max, name="", loc_mapping=None, group_mapping=None,
                 groups=None, ranges=None):
        self.solver = solver
        self._name = name
        # making sure, that if ranges are given, they are of the right dimension
        if ranges is None:
            ranges = [Range() for loc in range(n_poses)]
            for r in ranges:
                r.min = 10
                r.max = 500
        else:
            assert(isinstance(ranges, list))
            assert(len(ranges) == n_poses)
            for r in ranges:
                assert(isinstance(r, Range))


        self._visit_interval_vars = []
        self._visit_interval_vars = [self.solver.IntervalVar(0, time_max,  # type: list(IntervalVar) # start time
                                                             ranges[loc].min, ranges[loc].max,
                                                             # 10, 500,  # duration
                                                             0, time_max,  # end time
                                                             False,  # optional
                                                             "Interval_{:04d}_of_{:04d}_visit_".format(loc,
                                                                                                       n_poses) + name)
                                     for loc in range(n_poses)]

        self._spanning_visit_interval = self.solver.IntervalVar(0, time_max,  # type: list(IntervalVar) # start time
                                                                0, time_max,  # duration
                                                                0, time_max,  # end time
                                                                False,  # optional
                                                                "Spanning_Interval_" + name)
        #self._spanning_visit_interval.
        self.solver.Add(self._spanning_visit_interval.StartsAtStart(self._visit_interval_vars[0]))
        self.solver.Add(self._spanning_visit_interval.EndsAtEnd(self._visit_interval_vars[-1]))

        if self._visit_interval_vars.__len__() >= 2:
            for i1,i2 in zip(self._visit_interval_vars[:-1], self._visit_interval_vars[1:]):
                self.solver.Add(i2.StartsAfterEnd(i1))

        self._conf_connect_vars = [self.solver.IntVar(0, steps_max - 1, 'c_connect_' + name + "_{:04d}".format(n))
                                   for n in range(n_poses)]
        #
        # # letting the OVCs only connect to uneven configurations, making space for evasions
        # uneven_val = [2 * i + 1 for i in range(steps_max/2 + 1)]
        # for con_var in self._conf_connect_vars:
        #     con_var.SetValues(uneven_val)

        self._conf_values = [self.solver.IntVar(0, n_conf_max - 1, 'conf_' + name + "_{:04d}".format(n))
                             for n in range(n_poses)]

        nr_locs = max([max(loc_mapping[group].keys() + [0]) for group in group_mapping.keys()])
        self._loc_values = [self.solver.IntVar(0, nr_locs, 'loc_' + name + "_{:04d}".format(n))
                            for n in range(n_poses)]

        ex_group_domain_max = max(group_mapping.values())
        ex_group_domain_min = min(group_mapping.values())
        self._execution_group = self.solver.IntVar(ex_group_domain_min, ex_group_domain_max,
                                                   'execution_group_' + name)  # type: pywrapcp.IntVar
        self._execution_group.SetValues(group_mapping.values())

        cts = [LocConfCt(solver=solver, execution_group_var=self._execution_group, loc_var=self._loc_values[i],
                         conf_var=self._conf_values[i],
                         loc_mapping=loc_mapping,
                         group_mapping=group_mapping) for i in range(n_poses)]
        for ct in cts:
            solver.Add(ct)


        for i, loc in enumerate(self._visit_interval_vars):

            if i < len(self._visit_interval_vars) - 1:
                # Intervals must be in the given order
                self.solver.Add(self._visit_interval_vars[i + 1].StartsAfterEnd(self._visit_interval_vars[i]))

                # connection to configuration must be monotonic
                # self.solver.Add(self._conf_connect_vars[i] < self._conf_connect_vars[i + 1])
                # self.solver.Add(1 <= self._conf_connect_vars[i + 1] - self._conf_connect_vars[i] <= 1)
                self.solver.Add(self._conf_connect_vars[i] + 1 == self._conf_connect_vars[i + 1])
                # TODO: conf connect values should just be strictly monotonic and not

    def get_name(self):
        return self._name

    def add_dependency(self):
        pass

    def getInterval(self, interval=0):
        if interval == -1:
            return self._spanning_visit_interval
        try:
            return self._visit_interval_vars[interval]
        except KeyError:
            print("Don't have interval with index {}. Length of OVC is {}".format(interval, len(self._visit_interval_vars)))
            return None

    def get_var_list(self):
        var_list = []
        intervals = []

        for var in self._visit_interval_vars:
            intervals.append(var)
        intervals.append(self._spanning_visit_interval)
        var_list.append(self._execution_group)
        for var in self._conf_connect_vars + self._conf_values:
            var_list.append(var)
        return self._execution_group, self._conf_values, self._loc_values, self._conf_connect_vars, intervals
        # return var_list, intervals
