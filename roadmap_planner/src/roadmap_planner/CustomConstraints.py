#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import itertools
import math
from ortools.constraint_solver import pywrapcp
import numpy as np

from roadmap_planner.CustomDemons import DemonProp

from roadmap_planner.ordered_visiting_ct_var import OrderedVisitingConstraintVar

from roadmap_tools.prm import RoadMap


class IntervalBoundClashFailCt(pywrapcp.PyConstraint):

    def __init__(self, solver, interval_left, interval_right, spacial_collision):
        pywrapcp.PyConstraint.__init__(self, solver)
        self.__interval_left = interval_left  # type: IntervalVar
        self.__interval_right = interval_right  # type: IntervalVar
        self.__spatial_collision = spacial_collision

        self.__demon = []

    def Post(self):
        index = 0
        for interval_var in self.__interval_left:
            self.__demon.append(self.Demon(IntervalBoundClashFailCt.Update, index, "left"))
            interval_var.WhenAnything(self.__demon[-1])
            index += 1

        index = 0
        for interval_var in self.__interval_right:
            self.__demon.append(self.Demon(IntervalBoundClashFailCt.Update, index, "right"))
            interval_var.WhenAnything(self.__demon[-1])
            index += 1

    def InitialPropagate(self):
        pass

    def Update(self, index, component="left"):
        solver = self.solver()

        if component is "left":
            interval_var = self.__interval_left
            interval_var_other = self.__interval_right
        elif component is "right":
            interval_var = self.__interval_right
            interval_var_other = self.__interval_left

        bound = self.test_interval_bound(interval_var[index])
        if not bound:
            return

        index_other = 0
        for interval in interval_var_other:
            _bound = self.test_interval_bound(interval)
            if _bound:
                _temp_overlap = self.test_intervals_for_definit_overlap(interval_var[index], interval)
                if _temp_overlap:
                    if component is "left":
                        self.__spatial_collision[index][index_other].SetValue(0)
                    if component is "right":
                        self.__spatial_collision[index_other][index].SetValue(0)

    def test_interval_bound(self, interval):
        return interval.EndExpr().Bound() and interval.StartExpr().Bound()  # and interval.DurationExpr().Bound()

    def test_intervals_for_definit_overlap(self, i1, i2):
        # type: (IntervalVar, IntervalVar) -> bool
        i1_start = i1.StartExpr()
        i1_end = i1.EndExpr()

        i2_start = i2.StartExpr()
        i2_end = i2.EndExpr()

        i1_end_min = i1_end.Min()

        if i1_start.Max() <= i1_end.Min():
            if i1_start.Max() <= i2_start.Min() and i2_start.Max() <= i1_end.Min():
                return True
            elif i1_start.Max() <= i2_end.Min() and i2_end.Max() <= i1_end.Min():
                return True
        return False

class IntervalClashCt(pywrapcp.PyConstraint):
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
            self._demons.append(self.DelayedDemon(IntervalClashCt.Propagate, x, c_index))
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
        own_intervals = [2*index-1, 2*index, 2*index+1]
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

        # collecting the affected configurations
        affected_configurations = []
        not_affected_configurations = []

        affected_index = 0
        for c_o in self._c_other:
            if c_o.Bound():
                if c_o.Value() not in self._clash[val]:
                    affected_configurations.append(affected_index)
                else:
                    not_affected_configurations.append(affected_index)
            affected_index += 1

        affected_intervals = []
        for ind in affected_configurations:
            for i in range(2 * ind - 1, 2 * ind + 2, 1):
                if i < 0:
                    continue
                if i >= len(self._i_other):
                    continue
                affected_intervals.append(i)

        if affected_intervals.__len__() == 0:
            return

        affected_intervals = list(set(affected_intervals))

        disj_intervals_1 = []
        disj_intervals_2 = []
        for i in own_intervals:
            disj_intervals_1.append(self._i_self[i])
        for i in affected_intervals:
            disj_intervals_2.append(self._i_other[i])

        # for interval in disj_intervals_1:
        #     intervals = [interval]
        #     intervals.extend(disj_intervals_2)
        #     ct = solver.DisjunctiveConstraint(intervals, "disj_{}".format(interval))
        #     solver.AddConstraint(ct)

        # startVar = self._i_self[own_intervals[0]].StartExpr().Var()
        # endVar = self._i_self[own_intervals[-1]].EndExpr().Var()
        #
        # interval = solver.IntervalVar(startVar.Min(), startVar.Max(),
        #                               0, 1000,
        #                               endVar.Min(), endVar.Max(),
        #                               False,
        #                               "Interval_from_{}_to_{}".format(own_intervals[0], own_intervals[-1]))
        # # solver.AddConstraint(interval.StartExpr().Var() == startVar)
        # # solver.AddConstraint(interval.EndExpr().Var() == endVar)

        # it = itertools.product(own_intervals, affected_intervals)
        # for i, j in it:
        # # for i in own_intervals:
        #     # self._spacial_overlap[i][j].SetValue(1)
        #     ct = solver.DisjunctiveConstraint([self._i_self[i], self._i_other[j]], "disj_{}_{}".format(i, j))
        #     # ct = solver.DisjunctiveConstraint([self._i_self[i]] + disj_intervals_2, "disj_{}_others".format(i))
        #     # ct = solver.DisjunctiveConstraint([interval, self._i_other[j]], "disj_{}_{}".format(i, j))
        #     #ca = solver.ConstraintAdder(ct)
        #     # print "Intervals i_left_{} and i_right_{} should be disjunct".format(i, j)
        #     solver.AddConstraint(ct)
        for i in own_intervals_dict.keys():
            it = itertools.product(own_intervals_dict[i], affected_intervals_path[i])
            for intervals in it:
                ct = solver.TemporalDisjunction(intervals[0], intervals[1])
                solver.AddConstraint(ct)


                # it = itertools.product(disj_intervals_1, disj_intervals_2)
                # for intervals in it:
                #     ct = solver.TemporalDisjunction(intervals[0], intervals[1])
                #     solver.AddConstraint(ct)

            # ct = solver.DisjunctiveConstraint(disj_intervals_1 + disj_intervals_2, "disj_{}_others".format(i))
            # solver.AddConstraint(ct)

        # it = itertools.product(disj_intervals_1, disj_intervals_2)
        # for intervals in it:
        #     #print intervals
        #     if self.test_intervals_for_definit_overlap(intervals[0], intervals[-1]):
        #         print "colliding intervals are not disjunct"
        #         solver.Fail()
        #
        # solver.Add(solver.DisjunctiveConstraint(disj_intervals, "disj_{}_{}".format(index, val)))
        # # possibly, it can be stated as domain reductions on the intervals


class EndBound2UpperBoundCt(pywrapcp.PyConstraint):

    def __init__(self, solver, interval_vars, end_time_bound):
        pywrapcp.PyConstraint.__init__(self, solver)
        self.__interval_vars = interval_vars  # type: IntervalVar
        self.__end_time_bound = end_time_bound   # type: IntVar

    def Post(self):
        for interval_var in self.__interval_vars:
            demon_fixed_end = self.Demon(EndBound2UpperBoundCt.Update)
            interval_var.WhenEndBound(demon_fixed_end)

    def InitialPropagate(self):
        pass

    def Update(self):
        solver = self.solver()

        all_bound = True
        end_max = 0
        for interval_var in self.__interval_vars:
            if not interval_var.EndExpr().Bound():
                all_bound = False
            else:
                end_max = np.max([end_max, interval_var.EndMin()])

        if all_bound:
            self.__end_time_bound.SetMax(end_max)

            # Info: this is not useful, because constraints added during the search will be removed upon backtracking
            # solver.Add(self.__end_time_bound <= end_max)
            # solver.AddConstraint(self.__end_time_bound <= end_max)



class Edge2IntervalDurationCt(pywrapcp.PyConstraint):
    graph_with_min_times = {}

    def __init__(self, solver, interval_var, start_node, goal_node, roadmap):
        pywrapcp.PyConstraint.__init__(self, solver)
        self.__interval_var = interval_var  # type: IntervalVar
        self.__start = start_node   # type: IntVar
        self.__goal = goal_node     # type: IntVar
        self.__roadmap = roadmap  # type: RoadMap

    def Post(self):
        demon_start = self.DelayedDemon(Edge2IntervalDurationCt.Update_start)
        demon_goal = self.DelayedDemon(Edge2IntervalDurationCt.Update_goal)
        self.__start.WhenBound(demon_start)
        self.__goal.WhenBound(demon_goal)

    def InitialPropagate(self):
        pass

    def getMinDuration(self, start, goal):
        """
        function returning the min duration for between two roadmap nodes
        """
        SCALING_FACTOR = 10
        # dist in seconds
        if start == goal:
            return 0
        else:
            dist = self.__roadmap.get_edge_distance(start, goal)
            if dist == np.Inf:
                path = self.__roadmap.find_path_prm(start, goal, True)
                path_len = 0
                for i in range(len(path) - 1):
                    path_len += self.__roadmap.get_edge_distance(path[i], path[i + 1])
                return int(math.ceil(path_len * SCALING_FACTOR))
                # print "ups"
                # print self.__roadmap
            dist_int = int(math.ceil(dist * SCALING_FACTOR))
            return dist_int



    def Update_start(self):
        solver = self.solver()

        st_val = self.__start.Value()
        if self.__goal.Bound():
            goal_val = self.__goal.Value()
            if st_val == goal_val:  # if the traveling Interval is unused, it gets zero duration
                # self.__interval_var.SetDurationRange(1, 1)
                self.__interval_var.SetDurationRange(0, 0)
            elif st_val != goal_val:  # if the traveling interval is used, it gets the exact duration
                # slack is in the node intervals
                minDuration = self.getMinDuration(st_val, goal_val)
                if minDuration is np.Inf:
                    # TODO: hier ist was faul
                    print "minDuration is Inf zwischen {} und {}".format(st_val, goal_val)
                    solver.Fail()
                    return
                    # print "minDuration is Inf"
                    # solver.Fail()
                self.__interval_var.SetDurationRange(minDuration, minDuration)
        else:
            if not self.__goal.Contains(st_val):
                it = self.__goal.DomainIterator()
                dur = [self.getMinDuration(st_val, g_val) for g_val in it]
                self.__interval_var.SetDurationRange(np.min(dur), np.max(dur))

    def Update_goal(self):
        solver = self.solver()

        goal_val = self.__goal.Value()
        if self.__start.Bound():
            st_val = self.__start.Value()
            if st_val == goal_val:  # if the traveling Interval is unused, it gets zero duration
                # self.__interval_var.SetDurationRange(1, 1)
                self.__interval_var.SetDurationRange(0, 0)
            elif st_val != goal_val:  # if the traveling interval is used, it gets the exact duration
                # slack is in the node intervals
                minDuration = self.getMinDuration(st_val, goal_val)
                if minDuration is np.Inf:
                    print "minDuration is Inf zwischen {} und {}".format(st_val, goal_val)
                    # TODO: hier ist was faul
                    solver.Fail()
                    return
                    # solver.Fail()
                self.__interval_var.SetDurationRange(minDuration, minDuration)
        else:
            if not self.__start.Contains(goal_val):
                it = self.__start.DomainIterator()
                dur = [self.getMinDuration(s_val, goal_val) for s_val in it]
                self.__interval_var.SetDurationRange(np.min(dur), np.max(dur))


class InGoal2StatesCt(pywrapcp.PyConstraint):
    def __init__(self, solver, inGoal, vars, goal):
        pywrapcp.PyConstraint.__init__(self, solver)
        self.__in_goal = inGoal
        self.__vars = vars
        self.__goal = goal

    def Post(self):
        demon = self.Demon(InGoal2StatesCt.Update2)
        self.__in_goal.WhenBound(demon)

    def InitialPropagate(self):
        pass

    def Update2(self):
        solver = self.solver()
        inGoalValue = self.__in_goal.Value()

        if inGoalValue > len(self.__vars):
            solver.Fail()

        for i in range(-1, -inGoalValue - 1, -1):
            self.__vars[i].SetValue(self.__goal)


class TimeFeasibleCt(pywrapcp.PyConstraint):
    def __init__(self, solver, vars, graph, goal, intervals, final_time):
        pywrapcp.PyConstraint.__init__(self, solver)
        self.__vars = vars
        self.__graph = graph
        self.__goal = goal
        self.__intervals = intervals
        self.__final_time = final_time

    def Post(self):

        demon = self.Demon(TimeFeasibleCt.Update2, 0)
        self.__vars[0].WhenBound(demon)

        # for i in range(len(self.__vars)):
        #     v = self.__vars[i]
        #     if not v.Bound():
        #         demon = self.Demon(TimeFeasibleCt.Update, i)
        #         v.WhenBound(demon)

    def InitialPropagate(self):
        solver = self.solver()

        # for var in self.__vars:
        #     if var.Bound():
        #         path = self.GetShortestPath(var.Value(), self.__goal)
        #         if len(self.__vars) - self.__vars.index(var) < len(path):
        #             solver.Fail()

    def GetShortestPath(self, start, goal):
        path = shortest_path(self.__graph, self.__graph.vertex(start), self.__graph.vertex(goal))
        return path

    def GetMinTimePath(self, start, goal):
        path = self.GetShortestPath(start, goal)
        return (len(path)-1) * 5

    def Update2(self, index):
        solver = self.solver()
        value = self.__vars[index].Value()

        path = self.GetShortestPath(value, self.__goal)

        if len(self.__vars) - index < len(path[0]):
            solver.Fail()

        if len(self.__vars) - index == len(path[0]):
            for k, j in zip(range(index, len(self.__vars) - 1, 1), range(0, len(path) - 1, 1)):
                self.__vars[k].SetValue(int(path[0][j]))

    def Update(self, index):

        # print "In Update of PathFeasibleCt with var {}".format(self.__vars[index])
        solver = self.solver()
        value = self.__vars[index].Value()

        next_bound_index = -1
        next_bound = None

        for i in range(index + 1, self.__vars.__len__(), 1):
            if self.__vars[i].Bound():
                next_bound = self.__vars[i].Value()
                next_bound_index = i
                break

        available_steps = next_bound_index - index + 1
        path = self.GetShortestPath(value, next_bound)

        if available_steps < len(path[0]):
            # print "Too little steps"
            solver.Fail()

        if available_steps == len(path[0]):
            # print "set {} to {}".format(self.__vars[index:next_bound_index + 1], (path[0][:]))
            for k, j in zip(range(index, next_bound_index, 1), range(0, len(path) - 1, 1)):
                self.__vars[k].SetValue(int(path[0][j]))

        # find previous bound variable
        next_bound_index = -1
        next_bound = None

        for i in range(index - 1, -1, -1):
            if self.__vars[i].Bound():
                next_bound = self.__vars[i].Value()
                next_bound_index = i
                break

        available_steps = np.abs(index - next_bound_index) + 1
        path = self.GetShortestPath(next_bound, value)

        if available_steps < len(path[0]):
            solver.Fail()
            # print "Too little steps"

        if available_steps == len(path[0]):
            # print "set {} to {}".format(self.__vars[next_bound_index:index + 1], (path[0][:]))
            for k, j in zip(range(next_bound_index, index, 1), range(0, len(path) - 1, 1)):
                # print "set {} to {}".format(self.__vars[k], int(path[0][j]))
                self.__vars[k].SetValue(int(path[0][j]))


class PathFeasibleCt(pywrapcp.PyConstraint):
    def __init__(self, solver, vars_, prm):
        pywrapcp.PyConstraint.__init__(self, solver)
        self.__vars = vars_
        self.__prm = prm  # type: RoadMap
        # self.__dist_map = self.__prm.get_distances_to_node(0, len(self.__vars))
        self.__dist_map = prm.get_distances_to_node(0, len(self.__vars))
        self.__domain_max = set(range(prm.get_number_of_vertices()))

        self.__reachable_domain_cache = {}

    def Post(self):
        for i in range(len(self.__vars)):
            v = self.__vars[i]
            if not v.Bound():
                # demon = self.Demon(PathFeasibleCt.Update, i)
                demon = self.Demon(PathFeasibleCt.InitialPropagate)
                v.WhenBound(demon)

                # for i in range(0, len(self.__vars)):
            # demon = self.Demon(PathFeasibleCt.PropagateDomainToNeighbours, i)
                # demon = self.Demon(PathFeasibleCt.FailForIncompatibleNeighbours, i)
                # v.WhenBound(demon)

    def PropagateDomainToNeighbours(self, i):
        newDomain = set()

        var = self.__vars[i]
        # TODO: go on here
        it = var.DomainIterator()
        for val in it:
            newDomain.union(self.GetReachableDomain(val, 1))

        var.SetValues(list(newDomain))

    def FailForIncompatibleNeighbours(self, i):
        newDomain = set()

        var = self.__vars[i]
        # TODO: go on here
        reach_set = self.GetReachableDomain(var.Value(), 1)

        if i - 1 >= 0:
            self.__vars[i - 1].SetValues(reach_set)
        if i + 1 <= len(self.__vars):
            self.__vars[i + 1].SetValues(reach_set)

    def InitialPropagate(self):
        solver = self.solver()
        # TODO remove because not needed!!!
        index_bound = []
        values_bound = []
        distances_to_confs = []
        index = 0
        for var in self.__vars:
            if var.Bound():
                index_bound.append(index)
                values_bound.append(var.Value())
            index += 1

        distances_to_next_lower_bound_index = []
        distances_to_next_higher_bound_index = []
        steps_since_last_bound = -1
        for i in range(0, len(self.__vars)):
            if self.__vars[i].Bound():
                steps_since_last_bound = 0
            elif steps_since_last_bound == -1:
                steps_since_last_bound = -1
            else:
                steps_since_last_bound += 1
            distances_to_next_lower_bound_index.append(steps_since_last_bound)

        steps_since_last_bound = -1
        for i in range(len(self.__vars) - 1, -1, -1):
            if self.__vars[i].Bound():
                steps_since_last_bound = 0
            elif steps_since_last_bound == -1:
                steps_since_last_bound = -1
            else:
                steps_since_last_bound += 1
            distances_to_next_higher_bound_index.append(steps_since_last_bound)

        distances_to_next_higher_bound_index.reverse()

        # dist_map = self.__prm.get_distances_to_node(self.__vars[len(distances_to_next_lower_bound_index) - 1 -
        #                                                         distances_to_next_lower_bound_index[-1]].Value(),
        #                                             distances_to_next_lower_bound_index[-1])

        for index, distances_to_next in enumerate(distances_to_next_higher_bound_index):
            if distances_to_next == 0:
                this_val = self.__vars[index].Value()
                if index + 1 >= len(self.__vars):
                    # reached the last var
                    break
                if distances_to_next_higher_bound_index[index + 1] == -1:
                    # there is no higher bound value
                    break
                if distances_to_next_higher_bound_index[index + 1] == 0:
                    # the next value is already bound
                    break

                next_val_index = index + 1 + distances_to_next_higher_bound_index[index + 1]
                next_val = self.__vars[next_val_index].Value()

                available_values = next_val_index - index + 1

                path = self.GetShortestPath(this_val, next_val, use_weights=True)

                # TODO: reactivate assignment
                if len(path) == available_values:
                    print "{}: assign whole path from index {} path: {}".format(self.__prm.get_fingerprint(), index,
                                                                                path)
                    for path_index, new_var_val in enumerate(path):
                        var_index = index + path_index
                        self.__vars[var_index].SetValue(new_var_val)

                if len(path) > available_values:
                    print "{}: path too long ({}) for assignment: {}".format(self.__prm.get_fingerprint(), len(path),
                                                                             path)
                    solver.Fail()


        for i in range(0, len(self.__vars)):
            if self.__vars[i].Bound():
                continue
            domain_lower = []
            domain_higher = []
            domain = self.__domain_max
            if distances_to_next_lower_bound_index[i] > 0:
                index_lower = i - distances_to_next_lower_bound_index[i]
                value_lower = self.__vars[index_lower].Value()
                domain_lower = self.GetReachableDomain(value_lower, distances_to_next_lower_bound_index[i])
                domain = domain.intersection(domain_lower)
            if distances_to_next_higher_bound_index[i] > 0:
                index_higher = i + distances_to_next_higher_bound_index[i]
                value_higher = self.__vars[index_higher].Value()
                domain_higher = self.GetReachableDomain(value_higher, distances_to_next_higher_bound_index[i])
                domain = domain.intersection(domain_higher)

            # print domain
            if len(domain) == 0:
                print "Domain of {} is empty".format(self.__vars[i])
                values = []
                for k in range(len(self.__vars)):
                    if self.__vars[k].Bound():
                        values.append(self.__vars[k].Value())
                    else:
                        values.append(-1)
                print values
                print distances_to_next_higher_bound_index
                print distances_to_next_lower_bound_index
                print domain_higher
                print domain_lower
                solver.Fail()
            self.__vars[i].SetValues(domain)






            # new_domain = []
            # for index in range(len(distances_to_next_lower_bound_index)-1, -1, -1):
            #     if distances_to_next_lower_bound_index[index] == 0:
            #         if index > 0 and distances_to_next_lower_bound_index[index - 1] > 0:
            #             max_distance = distances_to_next_lower_bound_index[index - 1]
            #             focus_value = self.__vars[index - 1 - distances_to_next_lower_bound_index[index - 1]].Value()
            #             dist_map = self.__prm.get_distances_to_node(focus_value, max_distance)
            #             continue
            #         else:
            #             continue
            #     new_domain = []
            #     for val in self.__vars[index].DomainIterator():
            #         index_of_bound_var = index - distances_to_next_lower_bound_index[index]
            #         value_of_bound_var = self.__vars[index_of_bound_var].Value()
            #         dist = dist_map[value_of_bound_var][val]
            #         # print dist_map[val]
            #         if dist_map[value_of_bound_var][val] <= distances_to_next_lower_bound_index[index]:
            #             new_domain.append(val)
            #     self.__vars[index].SetValues(new_domain)
            #
            #
            #
            # distances_between_indexes = [index_bound[i] - index_bound[i-1] - 1 for i in range(1, len(index_bound))]
            # distances_between_indexes.insert(0, 0)
            # distances_between_indexes.append(0)
            #
            #
            #
            # for i in range(0, len(index_bound)):
            #     max_distance = np.max([distances_between_indexes[i], distances_between_indexes[i+1]])
            #     distances_to_confs.append(self.__prm.get_distances_to_node(values_bound[i], max_distance))
            #
            # print "here"


            # for var in self.__vars:
            #     if var.Bound():
            #         if var.Value() == self.__goal:
            #             continue
            #         path = self.GetShortestPath(var.Value(), self.__goal)
            #         if len(self.__vars) - self.__vars.index(var) < len(path):
            #             solver.Fail()

    def GetShortestPath(self, start, goal, use_weights=False):
        if start == goal:
            return []
        path = self.__prm.find_path_prm(start, goal, use_weights)
        if not path:
            print "Path is empty for start {} to goal {}. Group {}.".format(start, goal, self.__prm.get_fingerprint())
        # print path
        return path

    def GetReachableDomain(self, focus_value, max_dist):
        # type: (int, int) -> set[int]
        # max_distance = 15
        # focus_value = 409

        # try to get result from cache, otherwise calc it
        try:
            picked_vals = self.__reachable_domain_cache[(focus_value, max_dist)]
        except KeyError:
            max_distance = max_dist + 1

            dist_map = self.__dist_map

            gtnm = np.greater(len(dist_map[focus_value]) * [max_distance], dist_map[focus_value])
            picked_vals_dist = np.multiply(dist_map[focus_value], gtnm)
            picked_vals_map = np.greater(picked_vals_dist, len(dist_map[focus_value]) * [0])
            picked_vals_index = np.multiply(picked_vals_map, range(0, len(picked_vals_map), 1))
            picked_vals = set(picked_vals_index)

            if not picked_vals_map[0]:
                picked_vals.remove(0)
            picked_vals.add(focus_value)
            # save result to cache
            self.__reachable_domain_cache[(focus_value, max_dist)] = picked_vals

        return picked_vals

        # def Update2(self, index):
        #     solver = self.solver()
        #     value = self.__vars[index].Value()
        #
        #     if value == self.__goal:
        #         return
        #
        #     path = self.GetShortestPath(value, self.__goal)
        #
        #     if len(self.__vars) - index < len(path[0]):
        #         solver.Fail()
        #
        #     if len(self.__vars) - index == len(path[0]):
        #         for k, j in zip(range(index, len(self.__vars) - 1, 1), range(0, len(path) - 1, 1)):
        #             self.__vars[k].SetValue(int(path[0][j]))
        #
        # def Update(self, index):
        #
        #     # print "In Update of PathFeasibleCt with var {}".format(self.__vars[index])
        #     solver = self.solver()
        #     value = self.__vars[index].Value()
        #
        #     next_bound_index = -1
        #     next_bound = None
        #
        #     for i in range(index + 1, self.__vars.__len__(), 1):
        #         if self.__vars[i].Bound():
        #             next_bound = self.__vars[i].Value()
        #             next_bound_index = i
        #             break
        #
        #     available_steps = next_bound_index - index + 1
        #
        #     # if we don't have a final pose, there is in some cases no next_bound and therefore no path
        #     if next_bound is not None and value != next_bound:
        #         path = self.GetShortestPath(value, next_bound)
        #         if path is None:
        #             print "Path is None"
        #             solver.Fail()
        #
        #         if type(path) is int:
        #             print path
        #
        #         # TODO: check if what we get when there is no path?
        #         #
        #         if len(path) == 0:
        #             print "No paths found between {} and {}.".format(value, next_bound)
        #             return
        #
        #         if available_steps < len(path):
        #             # print "Too little steps"
        #             solver.Fail()
        #
        #         if available_steps == len(path):
        #             # print "set {} to {}".format(self.__vars[index:next_bound_index + 1], (path[0][:]))
        #             for k, j in zip(range(index, next_bound_index, 1), range(0, len(path) - 1, 1)):
        #                 self.__vars[k].SetValue(int(path[j]))
        #
        #     # find previous bound variable
        #     next_bound_index = -1
        #     next_bound = None
        #
        #     for i in range(index - 1, -1, -1):
        #         if self.__vars[i].Bound():
        #             next_bound = self.__vars[i].Value()
        #             next_bound_index = i
        #             break
        #
        #     available_steps = np.abs(index - next_bound_index) + 1
        #     if next_bound is not None and value != next_bound:
        #         path = self.GetShortestPath(next_bound, value)
        #
        #         if path is None:
        #             path2 = self.GetShortestPath(value, next_bound)
        #             solver.Fail()
        #             print "ups"
        #
        #         if not path:
        #             print "ups"
        #
        #         if available_steps < len(path):
        #             solver.Fail()
        #             print "Too little steps"
        #
        #         if available_steps == len(path):
        #             # print "set {} to {}".format(self.__vars[next_bound_index:index + 1], (path[0][:]))
        #             for k, j in zip(range(next_bound_index, index, 1), range(0, len(path) - 1, 1)):
        #                 # print "set {} to {}".format(self.__vars[k], int(path[0][j]))
        #                 self.__vars[k].SetValue(int(path[j]))


class MoveOnConnectedCompCt(pywrapcp.PyConstraint):
    def __init__(self, solver, x1, xn=[], graph={}):
        pywrapcp.Constraint.__init__(self, solver)
        self._g = graph
        self._x1 = x1
        self._xn = xn
        print 'Constraint built'

    def Post(self):
        print 'in Post()'
        self._demon = DemonProp(self._x1, self)
        # self._demon = Demon(self.InitialPropagate)
        self._x1.WhenBound(self._demon)
        # self._x1.WhenDomain(self._demon)
        print 'out of Post()'

    def Propagate(self):
        self.InitialPropagate()

    def InitialPropagate(self):
        print 'in InitialPropagate()'
        newDomain = set()
        for i in self._x1.DomainIterator():
            if i in self._g.keys():
                for val in self._g[i]:
                    newDomain.add(val)
                    # newDomain.add(i)
        newDomain_lst = list(newDomain)

        for x in self._xn:
            x.SetValues(newDomain_lst)

            # self._x.SetMin(5)
            # print self._x1
            # for x in self._xn:
            #     print x
            # print 'out of InitialPropagate()'


class ConnectedComponentCt(pywrapcp.PyConstraint):
    def __init__(self, solver, confs, prm):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._g = prm  # type: RoadMap
        self._x = confs
        # print 'Constraint built'
        # print self._x

    def Post(self):
        # print 'in Post()'
        self._demons = []
        index = 0
        for x in self._x:
            self._demons.append(self.Demon(ConnectedComponentCt.Propagate, index))
            x.WhenBound(self._demons[-1])
            index += 1
            # self._demon_r = DemonProp(self._xr, self)
            # self._demon = Demon(self.InitialPropagate)
            # self._xl.WhenBound(self._demon_l)
            # self._xr.WhenBound(self._demon_r)
            # self._x1.WhenDomain(self._demon)
            # print 'out of Post()'

    def Propagate(self, index):
        # newDomain = self.getConnectedNodes(self._x[index])
        newDomain = self._g.get_nodes_of_component(self._x[index].Value())

        for x in self._x:
            x.SetValues(newDomain)


    def InitialPropagate(self):
        # comp, hist = gt.label_components(g, attractors=False)
        # print 'in InitialPropagate()'
        # print 'out of InitialPropagate()'
        pass



class GraphClashCt(pywrapcp.PyConstraint):
    def __init__(self, solver, x_l, x_r, clash):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._clash = clash
        self._xl = x_l
        self._xr = x_r
        # print 'Constraint built'
        # print self._xl

    def Post(self):
        # print 'in Post()'
        # self._demon_l = CustomDemons.DemonProp(self._xl, self)
        demon_l = self.Demon(GraphClashCt.Propagate)
        # self._demon_r = DemonProp(self._xr, self)
        # self._demon = Demon(self.InitialPropagate)
        self._xl.WhenBound(demon_l)
        # self._xr.WhenBound(self._demon_r)
        # self._x1.WhenDomain(self._demon)
        # print 'out of Post()'

    def InitialPropagate(self):
        pass

    def Propagate(self):
        solver = self.solver()
        if self._xr.Bound() and self._xr.Value() not in self._clash[self._xl.Value()]:
            print "Fail for clash"
            solver.Fail()

        # newDomain = set()
        # for i in self._xl.DomainIterator():
        #     if i in self._clash.keys():
        #         for val in self._clash[i]:
        #             newDomain.add(val)
        # newDomain_lst = list(newDomain)
        # # TODO: better check if value is compatible with other value. If not -> Fail. Need xr clash for that
        #
        # self._xr.SetValues(newDomain_lst)

        # print self._xl
        # print self._xr


class MoveOnGraphCt(pywrapcp.PyConstraint):
    def __init__(self, solver, x1, xn, prm):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._prm = prm   # type: RoadMap
        self._x1 = x1
        self._xn = xn
        # print 'Constraint built'
        # print self._x1

    def Post(self):
        # print 'in Post()'
        # self._demon = DemonProp(self._x1, self)
        demon = self.Demon(MoveOnGraphCt.Propagate)
        # self._demon = Demon(self.InitialPropagate)
        self._x1.WhenBound(demon)
        # self._x1.WhenDomain(self._demon)
        # print 'out of Post()'

    def Propagate(self):
        # newDomain = set()

        # vertices = self._prm.vertices()
        # for i in self._x1.DomainIterator():
        #     if i in vertices:
                # for val in self._prm.get_neighbours(i):
                #     newDomain.add(val)
        #    newDomain = newDomain.union(set(self._prm.get_neighbours(i)))
        #    newDomain.add(i)

        # newDomain = set(self._prm.get_neighbours(self._x1.Value()))
        x1_value = self._x1.Value()
        newDomain = self._prm.get_neighbours(x1_value)
        newDomain.append(x1_value)
        # newDomain_lst = list(newDomain)

        for x in self._xn:
            x.SetValues(newDomain)

    def InitialPropagate(self):
        # print 'in InitialPropagate()'
        # newDomain = set()
        # for i in self._x1.DomainIterator():
        #     if i in self._g.keys():
        #         for val in self._g[i]:
        #             newDomain.add(val)
        #         newDomain.add(i)
        # newDomain_lst = list(newDomain)
        # print "initial Propagate"
        #########################################
        newDomain = set()

        # vertices = self._prm.vertices()
        for i in self._x1.DomainIterator():
            # if i in vertices:
                # for val in self._prm.get_neighbours(i):
                #     newDomain.add(val)
            newDomain = newDomain.union(set(self._prm.get_neighbours(i)))
            newDomain.add(i)
        newDomain_lst = list(newDomain)

        for x in self._xn:
            x.SetValues(newDomain_lst)
        ##########################################
            # self._x.SetMin(5)
            # print self._x1
            # for x in self._xn:
            #     print x
            # print 'out of InitialPropagate()'


class BindIntervalsCt(pywrapcp.PyConstraint):
    '''
    A constraint to keep intervals of motion series in sync with OVC intervals, when the respective connection variable
    gets bound.
    '''
    def __init__(self, solver, interval_vars, connect_vars, visit_intervals):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._interval_vars = interval_vars
        self._connect_vars = connect_vars
        self._visit_intervals = visit_intervals
        assert len(self._visit_intervals) == len(self._connect_vars)

    def Post(self):
        for i, var in enumerate(self._connect_vars):
            demon = self.Demon(BindIntervalsCt.Propagate, var, i)
            var.WhenBound(demon)

    def InitialPropagate(self):
        pass

    def Propagate(self, var, i):
        '''
        Keep task and motion interval in sync, when they get connected by a connection variable
        :param var: A connection variable associated with a task
        :param i: The index of the connection to identify the task interval
        :return: 0
        '''
        solver = self.solver()
        val = var.Value()
        try:
            ct = self._visit_intervals[i].StaysInSync(self._interval_vars[val * 2])
            solver.AddConstraint(ct)
        except IndexError:
            print("IndexError in BindIntervalsCt. Check consistency of roadmap and clash data!")




class IndexedEqualityCt(pywrapcp.PyConstraint):
    def __init__(self, solver, var_list, index_var, equal_val):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._var_list = var_list
        self._index_var = index_var
        self._equal_val = equal_val

    def Post(self):
        demon = self.Demon(IndexedEqualityCt.Propagate)
        self._index_var.WhenBound(demon)

        for i, var in enumerate(self._var_list):
            demon = self.DelayedDemon(IndexedEqualityCt.PropagateVarDomain, i)
            var.WhenDomain(demon)

    def InitialPropagate(self):
        pass

    def Propagate(self):
        solver = self.solver()
        self._var_list[self._index_var.Value()].SetValue(self._equal_val)
        # solver.AddConstraint(self._var_list[self._index_var.Value()] == self._equal_val)

    def PropagateVarDomain(self, i):
        if not self._var_list[i].Contains(self._equal_val):
            self._index_var.RemoveValue(i)


class IndexedEqualityVarCt(pywrapcp.PyConstraint):
    def __init__(self, solver, var_list, index_var, equal_var):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._var_list = var_list
        self._index_var = index_var
        self._equal_var = equal_var

    def Post(self):
        demon = self.Demon(IndexedEqualityVarCt.PropagateEqualVar)
        self._equal_var.WhenDomain(demon)


        demon = self.Demon(IndexedEqualityVarCt.Propagate)
        self._index_var.WhenBound(demon)

        for i, var in enumerate(self._var_list):
            demon = self.DelayedDemon(IndexedEqualityVarCt.PropagateVarDomain, i)
            var.WhenDomain(demon)

    def InitialPropagate(self):
        pass

    def Propagate(self):
        solver = self.solver()

        solver.AddConstraint(self._var_list[self._index_var.Value()] == self._equal_var)
        # self._var_list[self._index_var.Value()].SetValues(self._equal_val)
        # solver.AddConstraint(self._var_list[self._index_var.Value()] == self._equal_val)

    def PropagateVarDomain(self, i):
        # remove i from index_var domain, if the domains of equal_var and var_list[i] have no
        # common values
        if not self._index_var.Contains(i):
            return

        remove_var_from_index_var_domain = True
        for val in self._equal_var.DomainIterator():
            if self._var_list[i].Contains(val):
                remove_var_from_index_var_domain = False
                break
        if remove_var_from_index_var_domain:
            self._index_var.RemoveValue(i)

    def PropagateEqualVar(self):
        for i in self._index_var.DomainIterator():
            self.PropagateVarDomain(i)







class HorizonConfigurationCt(pywrapcp.PyConstraint):
    def __init__(self, solver, var_list, horizon):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._var_list = var_list
        self._horizon = horizon

    def Post(self):
        demon = self.Demon(HorizonConfigurationCt.Propagate)
        self._horizon.WhenBound(demon)

        for i, var in enumerate(self._var_list):
            demon = self.DelayedDemon(HorizonConfigurationCt.PropagateVarDomain, i)
            var.WhenDomain(demon)

    def InitialPropagate(self):
        pass

    def Propagate(self):
        # print self._horizon
        solver = self.solver()
        # solver.AddConstraint(self._var_list[self._index_var.Value()] == self._equal_val)
        # let everything past the horizon be equal to the final state
        for var in self._var_list[self._horizon.Value():]:
            if var.Bound():
                val = var.Value()
                for var_ in self._var_list[self._horizon.Value():]:
                    var_.SetValue(val)
                return

        for var in self._var_list[self._horizon.Value():-1]:
            solver.AddConstraint(var == self._var_list[-1])

    def PropagateVarDomain(self, i):

        # if not self._horizon.Contains(i):
        #     return
        if not self._var_list[-1].Bound():
            return

        final_value = self._var_list[-1].Value()
        if not self._var_list[i].Contains(final_value):
            self._horizon.RemoveValues(range(0, i + 1))


class BrushArmAssignment2PnPTasks(pywrapcp.PyConstraint):
    def __init__(self, solver, rp, wt_arm_assignment_vars, wt_brush_assignment_vars, wt_build_order_vars,
                 ordered_visiting_cst_vars):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._rp = rp  # type: RoadmapPlanner
        self._sosm = rp.sosm
        self._wt_arm_assignment_vars = wt_arm_assignment_vars
        self._wt_brush_assignment_vars = wt_brush_assignment_vars
        self._wt_build_order_vars = wt_build_order_vars
        # self._wt_build_order_dict = {}

        self._ordered_visiting_cst_vars = ordered_visiting_cst_vars

    def Post(self):

        # for i in range(0, len(self._wt_arm_assignment_vars)):
        #     demon = self.Demon(BrushArmAssignment2PnPTasks.Propagate, i)
        #     self._wt_brush_assignment_vars[i].WhenBound(demon)
        #     self._wt_arm_assignment_vars[i].WhenBound(demon)
        #     self._wt_build_order_vars[i].WhenBound(demon)

        for i in self._wt_arm_assignment_vars.keys():
            demon = self.Demon(BrushArmAssignment2PnPTasks.Propagate, i)
            self._wt_brush_assignment_vars[i].WhenBound(demon)
            self._wt_arm_assignment_vars[i].WhenBound(demon)

        for var in self._wt_build_order_vars:
            # self._wt_build_order_vars[i].WhenBound(demon)
            var.WhenBound(demon)

    def InitialPropagate(self):
        pass

    def Propagate(self, i):
        solver = self.solver()
        sosm = self._sosm

        # check if everything regarding that WT is bound
        for var in self._wt_build_order_vars:
            if not var.Bound():
                return
        # if not self._wt_build_order_vars[i].Bound():

        #     return
        if not self._wt_arm_assignment_vars[i].Bound():
            return
        if not self._wt_brush_assignment_vars[i].Bound():
            return

        wt_poses = sosm.get_alias_to_poses_for_type("plate")

        brush_poses = sosm.get_alias_to_poses_for_type("brush")

        tool_poses = sosm.get_alias_to_poses_for_type("torpedo")

        available_wts = sosm.get_available_objects_of_type("plate")

        available_tools = sosm.get_available_objects_of_type("torpedo")

        available_brushes = sosm.get_available_objects_of_type("brush")

        available_groups = {"right_arm": 2, "left_arm": 1}

        # check if poses are reachable
        arm_value = self._wt_arm_assignment_vars[i].Value()
        group_name = available_groups.keys()[available_groups.values().index(arm_value)]

        try:
            wt_poses[group_name][i]
        except KeyError:
            solver.Fail()

        try:
            brush_poses[group_name][self._wt_brush_assignment_vars[i].Value()]
        except KeyError:
            solver.Fail()

        # TODO: make this work with new version of alloc vars

        var = self._ordered_visiting_cst_vars[i]  # type: OrderedVisitingConstraintVar

        var._execution_group.SetValue(self._wt_arm_assignment_vars[i].Value())

        all_execution_groups_bound = True
        for visvar in self._ordered_visiting_cst_vars.values():
            if not visvar._execution_group.Bound():
                all_execution_groups_bound = False

        all_execution_order_bound = True
        for order_var in self._wt_build_order_vars:
            if not order_var.Bound():
                all_execution_order_bound = False

        if all_execution_groups_bound and all_execution_order_bound:
            it = itertools.combinations(self._ordered_visiting_cst_vars.values(), 2)
            for c in it:
                if c[0]._execution_group.Value() == c[1]._execution_group.Value():
                    # ct = solver.TemporalDisjunction(c[0]._spanning_visit_interval, c[1]._spanning_visit_interval)
                    # solver.AddConstraint(ct)
                    # group_name_prm = available_groups.keys()[available_groups.values().index(c[0]._execution_group.Value())]

                    idx0 = self._ordered_visiting_cst_vars.values().index(c[0])
                    key0 = self._ordered_visiting_cst_vars.keys()[idx0]
                    idx1 = self._ordered_visiting_cst_vars.values().index(c[1])
                    key1 = self._ordered_visiting_cst_vars.keys()[idx1]

                    idx0_first = True
                    for o_var in self._wt_build_order_vars:
                        if o_var.Value() in [key0, key1]:
                            if o_var.Value() == key0:
                                idx0_first = True
                                break
                            else:
                                idx0_first = False
                                break

                    if idx0_first:
                        ct = c[1]._spanning_visit_interval.StartsAfterEnd(c[0]._spanning_visit_interval)
                        solver.AddConstraint(ct)
                        # solver.AddConstraint(c[1]._conf_connect_vars[0] > c[0]._conf_connect_vars[-1])
                        solver.AddConstraint(c[0]._conf_connect_vars[-1] < c[1]._conf_connect_vars[0])

                    else:
                        ct = c[0]._spanning_visit_interval.StartsAfterEnd(c[1]._spanning_visit_interval)
                        solver.AddConstraint(ct)
                        # solver.AddConstraint(c[0]._conf_connect_vars[0] > c[1]._conf_connect_vars[-1])
                        solver.AddConstraint(c[1]._conf_connect_vars[-1] < c[0]._conf_connect_vars[0])

                        # solver.AddConstraint(ct)

        arm_value = self._wt_arm_assignment_vars[i].Value()
        group_name = available_groups.keys()[available_groups.values().index(arm_value)]

        # pre_wt_pose
        pre_wt_pose = None
        # if self._wt_build_order_vars[i].Value() == 0:
        for index, order_var in enumerate(self._wt_build_order_vars):
            if order_var.Value() == i and index == 0:
                pre_wt_pose = tool_poses[group_name].values()[0]
                break
                # if self._wt_build_order_vars[i].Value() == 0:
                #     pre_wt_pose = tool_poses[group_name].values()[0]
            elif order_var.Value() == i:
                prev = self._wt_build_order_vars[index - 1].Value()
                # if prev < 0:
                #     print "prev is not valid"
                # assert prev >= 0
                # prev_idx = -1
                # found_prev = False
                # while True:
                #     if prev < 0:
                #         print "prev is not valid"
                #     if found_prev:
                #         break
                idx_deduc = 1
                while True:
                    if prev == 3 or i == 3:
                        print "key error ahead!"
                        print prev
                        print i
                    if self._wt_arm_assignment_vars[prev].Value() == self._wt_arm_assignment_vars[i].Value():
                        try:
                            pre_wt_pose = wt_poses[group_name][prev]
                        except KeyError:
                            solver.Fail()
                        break
                    else:
                        idx_deduc += 1
                        if index - idx_deduc < 0:
                            pre_wt_pose = tool_poses[group_name].values()[0]
                            break
                        prev = self._wt_build_order_vars[index - idx_deduc].Value()

                break

                # # go through the order vars and search for the value that equals prev
                # # if that wt is assembled by the same arm, we know the pre_wt_pose
                # # otherwise we go on with decremented prev. if prev reaches -1 we know
                # # that the tool has to be taken from the table
                # for idx, order_val in enumerate(self._wt_build_order_vars):
                #     if prev < 0:
                #         print "prev is not valid"
                #     if order_val.Value() == prev:
                #         prev_idx = idx
                #         # assert prev_idx >= 0
                #         if self._wt_arm_assignment_vars[prev_idx].Value() == arm_value:
                #             if prev_idx + 1 not in wt_poses[group_name].keys():
                #                 solver.Fail()
                #             try:
                #                 pre_wt_pose = wt_poses[group_name][prev_idx + 1]
                #             except KeyError:
                #                 print "check if plus one is right"
                #             found_prev = True
                #             break
                #         # elif prev_idx == 0:
                #         elif prev == 0:
                #             pre_wt_pose = tool_poses[group_name].values()[0]
                #             found_prev = True
                #             break
                #         prev -= 1
                #         if prev < 0:
                #             print "prev is not valid"

        try:
            wt_pose = wt_poses[group_name][i]
        except KeyError:
            solver.Fail()
            print "key_error"
        brush_pose = brush_poses[group_name][self._wt_brush_assignment_vars[i].Value()]

        var._conf_values[0].SetValue(pre_wt_pose)
        var._conf_values[1].SetValue(wt_pose)
        var._conf_values[2].SetValue(brush_pose)
        var._conf_values[3].SetValue(wt_pose)

        path_1 = self._rp.roadmaps[group_name].find_path_prm(pre_wt_pose, wt_pose, use_weights=True)
        path_2 = self._rp.roadmaps[group_name].find_path_prm(wt_pose, brush_pose, use_weights=True)
        path_3 = self._rp.roadmaps[group_name].find_path_prm(brush_pose, wt_pose, use_weights=True)

        # solver.AddConstraint(var._conf_connect_vars[1] - var._conf_connect_vars[0] >= len(path_1) - 1)
        # solver.AddConstraint(var._conf_connect_vars[2] - var._conf_connect_vars[1] >= len(path_2) - 1)
        # solver.AddConstraint(var._conf_connect_vars[3] - var._conf_connect_vars[2] >= len(path_3) - 1)

        solver.AddConstraint(var._conf_connect_vars[1] - var._conf_connect_vars[0] >= 1)
        solver.AddConstraint(var._conf_connect_vars[2] - var._conf_connect_vars[1] >= 1)
        solver.AddConstraint(var._conf_connect_vars[3] - var._conf_connect_vars[2] >= 1)

        # solver.AddConstraint(var._conf_connect_vars[1] - var._conf_connect_vars[0] == len(path_1) - 1)
        # solver.AddConstraint(var._conf_connect_vars[2] - var._conf_connect_vars[1] == len(path_2) - 1)
        # solver.AddConstraint(var._conf_connect_vars[3] - var._conf_connect_vars[2] == len(path_3) - 1)

        # print var._conf_connect_vars

        conf_vars, interval_vars = self._rp.get_vars_for_group(group_name)

        solver.AddConstraint(BindIntervalsCt(solver, interval_vars, var._conf_connect_vars, var._visit_interval_vars))

        for k in range(0, 4):
            ct = IndexedEqualityCt(solver, conf_vars, var._conf_connect_vars[k], var._conf_values[k].Value())
            solver.AddConstraint(ct)



            # it = itertools.combinations(self._ordered_visiting_cst_vars, 2)
            #
            # for c in it:
            #     if not c[0]._execution_group.Bound() or not c[-1]._execution_group.Bound():
            #         continue
            #     if c[0]._execution_group.Value() == c[-1]._execution_group.Value():
            #         idx0 = self._ordered_visiting_cst_vars.index(c[0])
            #         idx1 = self._ordered_visiting_cst_vars.index(c[1])
            #
            #         if self._wt_build_order_vars[idx0] < self._wt_build_order_vars[idx1]:
            #             # ct = c[0]._spanning_visit_interval.EndsBeforeStart(c[1]._spanning_visit_interval)
            #             ct = c[1]._spanning_visit_interval.StartsAfterEnd(c[0]._spanning_visit_interval)
            #
            #         else:
            #             # ct = c[1]._spanning_visit_interval.EndsBeforeStart(c[0]._spanning_visit_interval)
            #             ct = c[0]._spanning_visit_interval.StartsAfterEnd(c[1]._spanning_visit_interval)
            #         solver.AddConstraint(ct)


class MinDiffAdjVisitCt(pywrapcp.PyConstraint):
    def __init__(self, solver, pre, post, prm):
        pywrapcp.PyConstraint.__init__(self, solver)
        self.__pre = pre
        self.__post = post
        self.__prm = prm  # type: RoadMap
        # self.__dist_map = self.__prm.get_distances_to_node(0, len(self.__vars))
        self.__dist_map = prm.get_distances_to_node(0, len(self.__vars))

    def Post(self):

        demon = self.Demon(MinDiffAdjVisitCt.Propagate)
        self.__pre.WhenBound(demon)
        self.__post.WhenBound(demon)

    def InitialPropagate(self):
        pass

    def GetShortestPath(self, start, goal, use_weights=False):
        if start == goal:
            return []
        path = self.__prm.find_path_prm(start, goal, use_weights)
        if not path:
            print "Path is empty for start {} to goal {}. Group {}.".format(start, goal, self.__prm.get_fingerprint())
        # print path
        return path

    def Propagate(self):
        pass


class VarContainerCt(pywrapcp.PyConstraint):
    def __init__(self, solver, intvars={}, intervalvars={}):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._intvars = intvars
        self._intervalvars = intervalvars

    def Post(self):
        print "saved variable: "
        print self._intvars
        print self._intervalvars
        pass

    def InitialPropagate(self):
        pass

        # Intended use was the following, but it did not work
        # if LOG_VARS:
        #     interval_vars = {i_s.Name(): i_s,
        #                      i_o.Name(): i_o}
        #     ct = VarContainerCt(solver, intervalvars=interval_vars)
        #     solver.Add(ct)


class CondOVCMonotonicCt(pywrapcp.PyConstraint):
    def __init__(self, solver, ovcs=[]):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._OVCs = ovcs

    def Post(self):
        demon = self.Demon(CondOVCMonotonicCt.InitialPropagate)
        for ovc in self._OVCs:
            ovc._execution_group.WhenBound(demon)

    def InitialPropagate(self):
        solver = self.solver()

        all_bound = True
        for ovc in self._OVCs:
            if not ovc._execution_group.Bound():
                all_bound = False
                return

        if all_bound and self._OVCs[0]._execution_group.Value() == self._OVCs[1]._execution_group.Value():
            solver.AddConstraint(self._OVCs[0]._conf_connect_vars[-1] < self._OVCs[1]._conf_connect_vars[0])


        # Intended use was the following, but it did not work
        # if LOG_VARS:
        #     interval_vars = {i_s.Name(): i_s,
        #                      i_o.Name(): i_o}
        #     ct = VarContainerCt(solver, intervalvars=interval_vars)
        #     solver.Add(ct)


class SameCompOVCConnectAllDifferent(pywrapcp.PyConstraint):
    def __init__(self, solver, ovcs=[]):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._OVCs = ovcs  # type: list[OrderedVisitingConstraintVar]
        self._sorted_connect_vars = {}

    def Post(self):
        for idx, ovc in enumerate(self._OVCs):
            demon = self.Demon(SameCompOVCConnectAllDifferent.Propagate, idx)
            ovc._execution_group.WhenBound(demon)

    def InitialPropagate(self):
        for idx, ovc in enumerate(self._OVCs):
            if ovc._execution_group.Bound():
                self.Propagate(idx)

    def Propagate(self, idx):
        solver = self.solver()  # type: pywrapcp.Solver

        own_ovc = self._OVCs[idx]
        own_group = own_ovc._execution_group.Value()
        vars = own_ovc.get_var_list()
        connect_vars = [] + vars[3]

        for ovc in self._OVCs:
            if ovc is own_ovc:
                continue
            if ovc._execution_group.Bound() and own_group == ovc._execution_group.Value():
                connect_vars += ovc.get_var_list()[3]

        #
        #
        # try:
        #     self._sorted_connect_vars[own_group] += connect_vars
        # except KeyError:
        #     self._sorted_connect_vars[own_group] = connect_vars

        ct = solver.AllDifferent(connect_vars)
        solver.AddConstraint(ct)

