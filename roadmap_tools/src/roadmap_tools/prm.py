#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import graph_tool.all as gt
from rs_vertex import rs_vertex
import numpy as np
import copy
from moveit_msgs.msg import RobotState, geometry_msgs
from moveit_commander import move_group

import itertools

from rospy import ServiceException

import file_handler
import datetime


class RoadMap:
    def __init__(self, mgroup=None, file_name="", group_name="right_arm", base_frame=None, eef_link=None, active_joints=None):
        self._group_name = group_name

        # mgroup.get_pose_reference_frame()
        # self._base_frame = base_frame
        self._fingerprint = ""  # type: str

        # self.fk = kinematics_interface.ForwardKinematics()

        self._g = gt.Graph()  # type: gt.Graph
        self._g.set_directed(False)

        # vertex property to hold the RobotState
        self._vert_conf = self._g.new_vertex_property("object")
        self._vert_name = self._g.new_vertex_property("string")
        self._vert_type = self._g.new_vertex_property("string")

        self._vert_pose = self._g.new_vertex_property("object")
        self._edge_dist = self._g.new_edge_property("float")
        self._edge_traj = self._g.new_edge_property("object")

        self._group_joint_names = self._g.new_graph_property("object")

        if mgroup is None:
            self._base_frame = base_frame
            self._eef_link = eef_link
            self.set_joint_names_for_group(active_joints)
        else:
            self._eef_link = mgroup.get_end_effector_link()
            self._base_frame = mgroup.get_pose_reference_frame()
            mgroup = mgroup  # type: move_group.MoveGroupCommander
            self.set_joint_names_for_group(mgroup.get_active_joints())

        # self.set_joint_names_for_group(mgroup.get_active_joints())

        self._path_cache = {}
        self._path_dist_cache = {}
        self._use_dist_in_paths = False
        self._neighbours_cache = {}
        self._vertice_cache = None
        self._edge_dist_cache = {}

        # caching of edge distances
        # self.get_edge_distance = mem.cache(self.get_edge_distance)

        if file_name is not "":
            self.load_from_file(file_name)

        i = 0
        for node in self.vertices():
            n = node  # type: gt.Vertex
            nh = n.all_neighbours()
            nl = []
            for n in nh:
                nl.append(int(n))
            self._neighbours_cache[i] = nl
            i += 1

            # index_set = range(0, self.get_number_of_vertices())
            # it = itertools.product(index_set, index_set)
            # for i1, i2 in it:
            #     self.find_path_prm(i1, i2)

    def get_eef_pose(self, v, fk=None):
        if type(v) == int:
            vert = self.get_vertex(v)
        else:
            vert = v

        try:
            pose = self._vert_pose[vert]
            if pose is None:
                raise KeyError
            return pose
        except KeyError:
            try:
                if fk is None:
                    raise EnvironmentError
                pose = fk.getFK(self._eef_link,
                                     self.get_joint_names_of_group(),
                                     self.get_joint_values(vert),
                                     self._base_frame)
            except ServiceException:
                print("ServiceException in FK call")
                pose = None

                print(self._eef_link,
                      self.get_joint_names_of_group(),
                      self.get_joint_values(vert),
                      self._base_frame)

                pose = fk.getFK(self._eef_link,
                                self.get_joint_names_of_group(),
                                self.get_joint_values(vert),
                                self._base_frame)

            self._vert_pose[vert] = pose
            return pose

    def get_joint_values(self, v):
        jv = []
        joint_names = self.get_joint_names_of_group()
        robot_state = self._vert_conf[v]
        for name in joint_names:
            try:
                index = robot_state.joint_state.name.index(name)
            except ValueError:
                continue
            jv.append(robot_state.joint_state.position[index])
        return jv

    def get_group_name(self):
        return self._group_name

    def set_edge_distance(self, e, dist):
        self._edge_dist[e] = dist

    def set_edge_traj(self, e, traj):
        self._edge_traj[e] = traj

    def get_edge_distance(self, start, goal):
        try:
            size = len(self._edge_dist_cache)
        except AttributeError:
            self._edge_dist_cache = {}
        try:
            dist = self._edge_dist_cache[(start, goal)]
        # except AttributeError:
        #     self._edge_dist_cache = {}
        except KeyError:
            try:
                dist = self._edge_dist_cache[(goal, start)]
            except KeyError:
                edge = self._g.edge(start, goal)
                if edge is None:
                    edge = self._g.edge(goal, start)
                if edge is None:
                    self._edge_dist_cache[(start, goal)] = np.Inf
                    return np.Inf

                dist = self._edge_dist[edge]
                self._edge_dist_cache[(start, goal)] = dist
        assert type(dist) == float
        return dist

    def set_joint_names_for_group(self, joint_names=[]):
        # type: (list[str]) -> bool

        if len(joint_names) > 0:
            self._group_joint_names[self._g] = joint_names
            return True
        else:
            return False

    def get_joint_names_of_group(self):
        return self._group_joint_names[self._g]

    def get_robot_state_for_vertex(self, vertex):
        if type(vertex) is int:
            v = self.get_vertex(vertex)
            if v is not None:
                try:
                    rs = self._vert_conf[v]
                    return rs
                except IndexError:
                    print "Cannot return robot_state for vertex {}".format(v)
                    # return self._vert_conf[v]
        else:
            return self._vert_conf[vertex]

    def vertices(self):
        return self._g.vertices()

    # def get_vertices(self):
    #     return self._g.get_vertices()

    def get_edges(self):
        return self._g.edges()

    def add_vertex(self, robot_state=None, name=None, obj_type=None):
        vert = self._g.add_vertex()
        if robot_state:
            assert isinstance(robot_state, RobotState)
            self._vert_conf[vert] = robot_state
        if name:
            self._vert_name[vert] = name
        if obj_type:
            self._vert_type[vert] = obj_type
        return vert

    def get_vertex_name(self, v):
        vert = self._g.vertex(v)

        name = self._vert_name[vert]
        return name

    def add_edge(self, v1, v2):
        return self._g.add_edge(v1, v2)

    def get_vertex_for_name(self, obj_name):
        for vert in self.vertices():
            if self._vert_name[vert] == obj_name:
                return vert

    def get_vertices_for_property(self, obj_type):

        obj_dict = {}
        type_array = []
        name_array = []
        for vert in self.vertices():
            # if self._vert_type[vert] in obj_type:
            if len(self._vert_type[vert]) > 0:
                obj_dict[self._vert_name[vert]] = {"type": self._vert_type[vert],
                                                   "rm_node": {self._group_name: int(vert)}}
            type_array.append(self._vert_type[vert])
            name_array.append(self._vert_name[vert])
        print type_array
        print name_array

        return obj_dict

    def get_type_for_vertex(self, vertex):
        if isinstance(vertex, int):
            vertex = self._g.vertex(vertex)
        assert isinstance(vertex, gt.Vertex)

        return self._vert_type[vertex]

    def clear_roadmap(self):
        self._g.clear()

    def load_from_file(self, file_name, format_version="old"):
        if format_version is "old":
            self.clear_roadmap()
            f = file_handler.file_handler()
            loaded_prm = f.load_prm(file_name)
            assert isinstance(loaded_prm["V"], dict)
            vert = loaded_prm["V"]  # type: dict[rs_vertex]
            last_vert = vert[vert.keys()[-1]]  # type: rs_vertex
            self._group_joint_names[self._g] = last_vert.joint_names
            self._group_name = last_vert.group_name
            for v in vert.keys():
                vertex = self.add_vertex()
                self._vert_conf[vertex] = vert[v].robot_state

            assert isinstance(loaded_prm["E"], dict)
            neighbours = loaded_prm["E"]  # type: dict[set]

            # assert type(loaded_prm["edges"]) is list[tuple[int, int]]
            isinstance(loaded_prm["edges"], list)
            edges = loaded_prm["edges"]  # type: list[tuple[int, int]]
            for e in edges:
                v1 = self.get_vertex(e[0])
                v2 = self.get_vertex(e[1])
                self.add_edge(v1, v2)
            self._fingerprint = file_name
            return

    def delete_edge(self, edge):
        # type: (Graph.edge) -> bool
        return self._g.remove_edge(edge)

    def get_number_of_edges(self):
        return self._g.edge_index_range

    def get_vertex(self, index):
        return self._g.vertex(index)

    def get_number_of_vertices(self):
        # type: () -> int
        it = self._g.vertices()
        nr = 0
        for v in it:
            nr += 1
        return nr

    def generate_fingerprint(self, fingerprint=""):
        if fingerprint == "":
            self._fingerprint = "prm_" + self._group_name + "_" + str(datetime.datetime.today())
        else:
            self._fingerprint = fingerprint
        return self._fingerprint

    def get_fingerprint(self):
        if self._fingerprint is "":
            self.generate_fingerprint()
        return self._fingerprint

    def vis_prm(self, name="prm"):
        gt.graph_draw(self._g, vertex_text=self._g.vertex_index, vertex_font_size=2,
                                  vertex_shape="double_circle",
                                  vertex_fill_color="#729fcf", vertex_pen_width=1,
                                  output="{0}.pdf".format(name))

    def get_distances_to_node(self, target, max_dist):
        if type(target) is int:
            target = self._g.vertex(target)
        dist_map = gt.shortest_distance(self._g, None, self._g.vertex(target), max_dist=max_dist)
        return dist_map

    def shortest_distance(self, g=None, source=None, target=None, weights=None,
                          negative_weights=False, max_dist=None, directed=None,
                          dense=False, dist_map=None, pred_map=False):
        if g is None:
            g = self._g
        if weights:
            weights = self._edge_dist
        return gt.shortest_distance(g, source, target, weights, negative_weights, max_dist, directed, dense, dist_map,
                                    pred_map)

    def calc_path_distance(self, path):
        assert isinstance(path, list)
        if len(path) < 2:
            return 0

        try:
            dist = self._path_dist_cache[tuple(path)]
        except AttributeError:
            self._path_dist_cache = {}
            dist = 0
            for pos1, pos2 in zip(path[:-1], path[1:]):
                # dist += self.shortest_distance(source=pos1, target=pos2, weights=True)
                dist += self.get_edge_distance(start=pos1, goal=pos2)
            self._path_dist_cache[tuple(path)] = dist
        except KeyError:
            dist = 0
            for pos1, pos2 in zip(path[:-1], path[1:]):
                # dist += self.shortest_distance(source=pos1, target=pos2, weights=True)
                dist += self.get_edge_distance(start=pos1, goal=pos2)
            self._path_dist_cache[tuple(path)] = dist

        return dist

    def find_path_prm(self, start, goal, use_weights=False):
        # dist = graph_tool.all.shortest_distance(self._g, self._g.vertex(start), self._g.vertex(goal))
        # start_time = time.time()
        # path = graph_tool.all.shortest_path(self._g, self._g.vertex(start), self._g.vertex(goal))
        # elapsed_time = (time.time() - start_time)
        # f = elapsed_time
        # print "Time to calc path: %f" % f
        # p = path[0]
        # p2 = []
        # p2 = list(np.array(p, dtype=int))
        # return p2

        if start == goal:
            return []

        if type(start) is not int or type(goal) is not int:
            start = int(start)
            goal = int(goal)

        weights = None

        if use_weights and not self._use_dist_in_paths:
            print "Dropping path cache, because we use distance now."
            self._use_dist_in_paths = True
            self._path_cache = {}
        elif not use_weights and self._use_dist_in_paths:
            print "Dropping path cache, because we don't use distances anymore."
            self._use_dist_in_paths = False
            self._path_cache = {}

        if use_weights:
            weights = self._edge_dist

        if start == goal:
            pass
            # print "Start equals goal. Path will be empty."
        # assert goal != start
        try:
            path = self._path_cache[(start, goal)]
            # path_assert = list(
            #     np.array(graph_tool.all.shortest_path(self._g, self._g.vertex(start), self._g.vertex(goal))[0],
            #              dtype=int))
            # if path_assert != path:
            #     assert path == path_assert
            if len(path) < 2:
                raise KeyError
            return path
        except KeyError:
            pass
        try:
            path = copy.copy(self._path_cache[(goal, start)])  # type: list
            path.reverse()

            # path_assert = list(
            #     np.array(graph_tool.all.shortest_path(self._g, self._g.vertex(start), self._g.vertex(goal))[0],
            #              dtype=int))
            # if path_assert != path:
            #     assert path == path_assert
            if len(path) < 2:
                raise KeyError
            return path
        except KeyError:
            pass

        # path = list(
        #     np.array(graph_tool.all.shortest_path(self._g, self._g.vertex(start), self._g.vertex(goal))[0], dtype=int))
        # path_assert = list(
        #     np.array(graph_tool.all.shortest_path(self._g, self._g.vertex(goal), self._g.vertex(start))[0], dtype=int))
        # path_assert.reverse()
        # assert path == path_assert

        path = list(np.array(gt.shortest_path(self._g, self._g.vertex(start), self._g.vertex(goal), weights=weights)[0],
                    dtype=int))

        if len(path) < 2:
            print("path invalid: shorter than 2 for start != goal.")
            component_start = self.get_nodes_of_component(start)
            component_goal = self.get_nodes_of_component(goal)
            if goal not in component_start:
                print("nodes {} and {} are not connected. Check Roadmap!".format(start, goal))

        # if path[0] == 46 and path[-1] == 135:
        #     print "here"
        #
        # if path[0] != start or path[-1] != goal:
        #     print "path[0] != start or path[-1] != goal"
        self._path_cache[(start, goal)] = path
        # if not path:
        #     print "Path is empty. Something is wrong."

        return path

        # if (start, goal) in self._path_cache.keys():
        #     return self._path_cache[(start, goal)]
        # elif (goal, start) in self._path_cache.keys():
        #     path = self._path_cache[(goal, start)]  # type: list
        #     return path.reverse()
        # else:
        #     path = list(np.array(graph_tool.all.shortest_path(self._g, self._g.vertex(start), self._g.vertex(goal))[0], dtype=int))
        #     self._path_cache[(start, goal)] = path
        #     return path

        # return list(np.array(graph_tool.all.shortest_path(self._g, self._g.vertex(start), self._g.vertex(goal))[0], dtype=int))

        # for v in p:
        #     p2.append(int(v))
        # return p2

    def get_component_details(self):
        comp, hist = gt.label_components(self._g, attractors=False)

        return comp, hist

    def get_nodes_of_major_component(self):
        comp, hist = self.get_component_details()
        # mj_comp_id = hist.index(max(hist))
        mj_comp_id = np.where(hist == max(hist))[0][0]
        return self.get_nodes_for_component_id(mj_comp_id)

    def get_nodes_of_component(self, node):
        if isinstance(node, gt.Vertex):

            node = int(node)
        comp, hist = gt.label_components(self._g, attractors=False)
        group = comp.a[node]
        nodes = []
        for i in range(0, len(comp.a), 1):
            if comp.a[i] == group:
                nodes.append(i)

        return nodes

    def get_nodes_for_component_id(self, c_id):
        comp, hist = gt.label_components(self._g, attractors=False)
        if c_id >= len(hist):
            print("Received invalid id ({}). PRM has only {} components.".format(c_id, len(hist)))
            return []

        # TODO: can be implemented more efficiently with numpy.
        vs = []
        for v in self.vertices():
            if comp.a[int(v)] == c_id:
                vs.append(v)
                if len(vs) >= hist[c_id]:
                    break
        return vs

    def get_degree(self, node):
        # type: (gt.Vertex) -> int
        assert isinstance(node, gt.Vertex)
        out_degree = node.out_degree()
        in_degree = node.in_degree()
        degree = out_degree + in_degree
        return degree

    def get_neighbours(self, vertex):
        assert type(vertex) is int
        assert vertex <= self._g.vertex_index
        try:
            return self._neighbours_cache[vertex]
        except KeyError:
            print "Vertex {} is not in cache"

        if vertex in self._neighbours_cache.keys():
            return self._neighbours_cache[vertex]
        else:
            v = self._g.vertex(vertex)  # type: gt.Vertex
            nh = v.all_neighbours()
            # self._g.get_out_neighbours(vertex)
            nl = []
            for n in nh:
                nl.append(int(n))
            return nl

            # nh = self._g.get_in_neighbours(vertex)
            # nh.extend(self._g.get_out_neighbours(vertex))
