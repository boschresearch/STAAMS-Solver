#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from ortools.constraint_solver import pywrapcp


class LocConfCt(pywrapcp.PyConstraint):
    def __init__(self, solver, loc_var, conf_var, execution_group_var, loc_mapping, group_mapping):
        pywrapcp.PyConstraint.__init__(self, solver)
        self._loc_var = loc_var
        self._conf_var = conf_var
        self._execution_group_var = execution_group_var

        self._lambda = loc_mapping
        self._gm = group_mapping  # type: dict[str, int]

        print self

    def Post(self):
        demon = self.Demon(LocConfCt.PropagateLocDomain)
        self._loc_var.WhenDomain(demon)
        # self._execution_group_var.WhenDomain(demon)

        # self._loc_var.WhenBound(demon)
        demon = self.Demon(LocConfCt.PropagateLocDomain)
        self._execution_group_var.WhenBound(demon)

        demon = self.Demon(LocConfCt.PropagateConfDomain)
        self._conf_var.WhenDomain(demon)
        # self._conf_var.WhenBound(demon)
        # self._execution_group_var.WhenDomain(demon)

        # self._conf_var.WhenBound(demon)
        demon = self.Demon(LocConfCt.PropagateConfDomain)
        self._execution_group_var.WhenDomain(demon)
        # self._execution_group_var.WhenBound(demon)

    def InitialPropagate(self):
        self.PropagateLocDomain()
        self.PropagateConfDomain()

        # print self.DebugString()

    def PropagateLocDomain(self):
        groups = []
        for group_id in self._execution_group_var.DomainIterator():
            groups.append(self._gm.keys()[self._gm.values().index(group_id)])

        conf_domain = set()
        group_domain = set()
        for gn in groups:
            for loc in self._loc_var.DomainIterator():
                try:
                    conf_domain.add(self._lambda[gn][loc])
                    group_domain.add(self._gm[gn])
                except KeyError:
                    pass

        # print conf_domain
        if conf_domain.__len__() == 0:
            print "conf_domain empty"
        if group_domain.__len__() == 0:
            print "group_domain empty"
        self._conf_var.SetValues(conf_domain)
        self._execution_group_var.SetValues(group_domain)

    def PropagateConfDomain(self):
        groups = []
        for group_id in self._execution_group_var.DomainIterator():
            groups.append(self._gm.keys()[self._gm.values().index(group_id)])

        loc_domain = set()
        group_domain = set()
        for gn in groups:
            for conf in self._conf_var.DomainIterator():
                try:
                    idx = self._lambda[gn].values().index(conf)
                    loc_domain.add(self._lambda[gn].keys()[idx])
                    group_domain.add(self._gm[gn])
                except ValueError:
                    pass

        # print loc_domain
        if loc_domain.__len__() == 0:
            print "loc_domain empty"
        if group_domain.__len__() == 0:
            print "group_domain empty"
        self._loc_var.SetValues(loc_domain)
        self._execution_group_var.SetValues(group_domain)

