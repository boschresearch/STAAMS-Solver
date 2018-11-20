#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from ortools.constraint_solver import pywrapcp
import graph_tool.all as gt
import random


class DependencyDecisionBuilder(pywrapcp.PyDecisionBuilder):
    """
    Decision Builder which takes the dependencies between tasks into consideration
    """

    def __init__(self):
        pywrapcp.PyDecisionBuilder.__init__(self)

        self.__g = gt.Graph(directed=True)
        self.__vert_name = self.__g.new_vertex_property("string")
        self.__vert_var = self.__g.new_vertex_property("object")
        self.__dependency_type = self.__g.new_edge_property("int")
        self.__name_to_node = {}

    def addDependency(self, name, depends_on, dependency_type=1):
        if isinstance(name, pywrapcp.IntVar) and isinstance(depends_on, pywrapcp.IntVar):
            name = name.Name()
            depends_on = depends_on.Name()
        if name not in self.__name_to_node.keys() or depends_on not in self.__name_to_node.keys():
            raise ValueError
        self.__g.add_edge(self.__name_to_node[name], self.__name_to_node[depends_on])

    def addTask(self, var):
        # type: (pywrapcp.IntVar) -> None
        v = self.__g.add_vertex()
        self.__vert_name[v] = var.Name()
        self.__name_to_node[var.Name()] = v
        self.__vert_var[v] = var

    def Next(self, solver):
        """ selecting the mit value from to schedule the task as early as possible """
        # type: pywrapcp.Solver -> pywrapcp.Decision
        var = self.NextVar()
        if var:
            decision = solver.AssignVariableValue(var, var.Min())
            return decision
        else:
            return None

    def NextVar(self):
        """ selecting a random variable without unscheduled dependencies """

        # collect all variables that have no open dependencies
        no_deps = []
        for v in self.__g.vertices():
            if self.__vert_var[v].Bound():
                continue
            is_leaf = True
            neighbors = self.__g.get_out_neighbors(v)
            for n in neighbors:
                if not self.__vert_var[n].Bound():
                    is_leaf = False
            if is_leaf:
                no_deps.append(self.__vert_var[v])

        if len(no_deps) > 0:
            return no_deps[random.randint(0, len(no_deps) - 1)]
        else:
            return None

    def DrawDependencies(self):
        gt.graph_draw(self.__g, vertex_text=self.__vert_name, vertex_font_size=2,
                      vertex_shape="double_circle",
                      vertex_fill_color="#729fcf", vertex_pen_width=1,
                      output="{0}.pdf".format("dependency_graph"))

    def DebugString(self):
        var_list_str = ""
        for v in self.__g.vertices():
            if var_list_str.__len__() == 0:
                var_list_str += self.__vert_var[v].DebugString()
            else:
                var_list_str += ', ' + self.__vert_var[v].DebugString()

        return 'DependencyDecisionBuilder(' + str(self.__g) + ' - ' + var_list_str + ')'
