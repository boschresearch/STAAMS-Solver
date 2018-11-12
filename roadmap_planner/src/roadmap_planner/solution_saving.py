#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import inspect

from ortools.constraint_solver import pywrapcp
from ordered_visiting_ct_var import OrderedVisitingConstraintVar


class Variable:
    def __init__(self):
        self.type = None
        self.name = None

    def __str__(self):
        # members = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("__")]
        attributes = inspect.getmembers(self, lambda a: not (inspect.isroutine(a)))
        d = [a for a in attributes if not (a[0].startswith('__') and a[0].endswith('__'))]
        s = ""
        for v in d:
            s += str(v)
        return s


class IntVariable(Variable):
    def __init__(self, variable):
        self.type = pywrapcp.IntVar
        self.name = variable.Name()
        self.value = variable.Value()


class IntervalVariable(Variable):
    def __init__(self, ortools_interval_var):
        assert type(ortools_interval_var) == pywrapcp.IntervalVar
        self.type = pywrapcp.IntervalVar
        value = ortools_interval_var
        self.start_min = value.StartExpr().Min()
        self.start_max = value.StartExpr().Max()
        self.end_min = value.EndExpr().Min()
        self.end_max = value.EndExpr().Max()
        self.duration_min = value.DurationExpr().Min()
        self.duration_max = value.DurationExpr().Max()
        self.performed_min = value.PerformedExpr().Min()
        self.performed_max = value.PerformedExpr().Max()


class OrderedVisitingConstraint(Variable):
    def __init__(self, var):
        self.type = OrderedVisitingConstraintVar
        self.name = "OVC_" + var._name
        self._visit_interval_vars = [IntervalVariable(intvvar) for intvvar in var._visit_interval_vars]
        self._spanning_visit_interval = IntervalVariable(var._spanning_visit_interval)
        self._conf_connect_vars = [IntVariable(intvar) for intvar in var._conf_connect_vars]
        self._conf_values = [IntVariable(intvar) for intvar in var._conf_values]
        self._execution_group = IntVariable(var._execution_group)

    def __str__(self):
        # members = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("__")]

        s = ""

        s += self.name.__str__()
        s += self.concat_str_list([t.__str__() for t in self._visit_interval_vars])
        s += self._spanning_visit_interval.__str__()
        s += self.concat_str_list([t.__str__() for t in self._conf_connect_vars])
        s += self.concat_str_list([t.__str__() for t in self._conf_values])
        s += self._execution_group.__str__()

        s += "\n"
        return s

    def concat_str_list(self, l=None):
        if not l:
            return ""
        s = ""
        for t in l:
            s += t
            s += "\n"
        return s


class SolverStats:
    def __init__(self, solver=None):
        if solver:
            self.time = solver.WallTime()
            self.failures = solver.Failures()
            self.branches = solver.Branches()
            self.depth = solver.SearchDepth()
