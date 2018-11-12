#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from ortools.constraint_solver import pywrapcp


class DemonProp(pywrapcp.PyDemon):
    def __init__(self, x, ct):
        pywrapcp.PyDemon.__init__(self)
        self._x = x
        self._ct = ct
        # print 'Demon built'

    def Run(self, solver):
        # print 'in Run(), saw ' + str(self._x)
        # it = self._x.DomainIterator()
        # print "Domain of x:"
        # for i in it:
        #     print i

        self._ct.Propagate()


class DemonProp2(pywrapcp.PyDemon):
    def __init__(self, x, ct):
        pywrapcp.PyDemon.__init__(self)
        self._x = x
        self._ct = ct
        # print 'Demon built'

    def Run(self, solver):
        # print 'in Run(), saw ' + str(self._x)
        # it = self._x.DomainIterator()
        # print "Domain of x:"
        # for i in it:
        #     print i

        self._ct.Propagate(self._x)


class DemonTest(pywrapcp.PyDemon):
    def __init__(self, x):
        pywrapcp.PyDemon.__init__(self)
        self._x = x
        print 'Demon built'

    def Run(self, solver):
        print 'in Run(), saw ' + str(self._x)
        it = self._x.DomainIterator()
        print "Domain of x:"
        for i in it:
            print i