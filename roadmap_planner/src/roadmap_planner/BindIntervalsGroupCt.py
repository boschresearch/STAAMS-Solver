#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from ortools.constraint_solver import pywrapcp


class BindIntervalsGroupCt(pywrapcp.PyConstraint):
    '''
    A constraint which adds a set of constraints to the problem when the variable group_var gets bound.
    '''
    def __init__(self, solver, group_var, constraints):
        pywrapcp.PyConstraint.__init__(self, solver)

        self.group_var = group_var
        self.constraints = constraints

    def Post(self):
        demon = self.Demon(BindIntervalsGroupCt.Propagate)
        self.group_var.WhenBound(demon)

    def InitialPropagate(self):
        '''
        If the variable self.group_var is already bound, Propagate is immediately called
        :return:
        '''
        if self.group_var.Bound():
            self.Propagate()

    def Propagate(self):
        '''
        Adds the constraints self.constraints to the solver
        :return: 0
        '''
        solver = self.solver()  # type: pywrapcp.Solver
        group = self.group_var.Value()

        for ct in self.constraints[group]:
            solver.AddConstraint(ct)
