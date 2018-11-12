#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from roadmap_tools.prm_factory import RoadMapFactory
import rospy
import visualization_msgs.msg
from moveit_msgs.msg import geometry_msgs

from roadmap_tools.prm import RoadMap
import copy
import datetime


class RoadMapVisualizer:
    def __init__(self, rm_name=None, rm_path=None):
        self.rm_name = rm_name
        if rm_path:
            self.prm = RoadMapFactory.load_from_file(rm_path)
        elif rm_name:
            self.prm = RoadMapFactory.load_prm_from_database(rm_name)  # type: RoadMap
        self.group_name = self.prm.get_group_name()
        rospy.init_node("RoadMapVisualizer" + self.group_name)

        self.group_name = self.prm.get_group_name()

        self.marker_publisher = rospy.Publisher('prm_markers', visualization_msgs.msg.MarkerArray, queue_size=1000)

        self.V_viz_open = set()
        self.E_viz_open = set()
        self.no_subscribers_viz = 0
        self.V_poses = {}
        self.last_visualized = datetime.datetime.now()

        self.highlights = []

        self.timer = rospy.Timer(rospy.Duration(0.1), self.handle_visualize_timer)
        rospy.on_shutdown(self.shutdown_hook)

        rospy.spin()

    def handle_visualize_timer(self, e):
        rospy.logdebug("In handle_visualize_timer")
        no_subscribers = self.marker_publisher.get_num_connections()
        if no_subscribers < 1:
            self.no_subscribers_viz = no_subscribers
            return

        # rospy.loginfo("There is " + str(no_subscribers) + " Subscriber. Before: " + str(self.no_subscribers_viz))

        if self.no_subscribers_viz < no_subscribers or (
                    datetime.datetime.now() - self.last_visualized).total_seconds() > 25.0:
            self.last_visualized = datetime.datetime.now()
            self.add_all_nodes_for_viz()
            self.add_all_edges_for_viz()
            # self.V_viz_open.clear()
            # for key in self.V.keys():
            #     self.V_viz_open.add(key)
            # print self.V_viz_open
            # self.E_viz_open.clear()
            # for edge in self.edges:
            #     self.E_viz_open.add(self.edges.index(edge))
            # print self.E_viz_open
        self.no_subscribers_viz = no_subscribers

        rospy.wait_for_service('compute_fk')
        self.visualize_edges(self.highlights)
        self.visualize_prm(self.highlights)

    def visualize_prm(self, highlight=[], action=visualization_msgs.msg.Marker.ADD):

        marker_array = visualization_msgs.msg.MarkerArray()

        node_set = self.V_viz_open.copy()

        # for conf in self.V.keys():
        for conf in node_set:  # type: Vertex
            self.V_viz_open.remove(conf)
            # TODO: only vis new configurations and cache poses
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = self.prm._base_frame  # _"WAIST"
            # marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "prm_poses_" + self.group_name
            marker.id = int(conf)

            marker.type = visualization_msgs.msg.Marker.ARROW
            marker.action = action

            # eef_pose = self.get_pose(self.V[conf])
            eef_pose = self.prm.get_eef_pose(conf)
            pos = eef_pose.pose_stamped[0]
            marker.pose.position.x = pos.pose.position.x
            marker.pose.position.y = pos.pose.position.y
            marker.pose.position.z = pos.pose.position.z
            marker.pose.orientation.x = pos.pose.orientation.x
            marker.pose.orientation.y = pos.pose.orientation.y
            marker.pose.orientation.z = pos.pose.orientation.z
            marker.pose.orientation.w = pos.pose.orientation.w
            marker.scale.x = 0.05
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            if conf in highlight:
                marker.color.a = 0.9
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif len(self.prm.get_vertex_name(conf)) > 0:
                # for confs with name
                marker.color.a = 0.8
                marker.color.r = 1.0
                marker.color.g = 0.2
                marker.color.b = 0.0
                marker.ns = "prm_object_" + self.group_name

            # elif conf in self.E.keys():
            elif conf.out_degree() + conf.in_degree() > 0:
                marker.color.a = 0.3
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.a = 0.6
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            # make a marker for the node number
            text_marker = copy.deepcopy(marker)
            text_marker.ns = "prm_poses_numbers" + self.group_name
            text_marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
            text_marker.text = str(conf)

            # Text position
            text_marker.pose.position.z = text_marker.pose.position.z - 0.02

            # Text scale
            text_marker.scale.z = 0.02

            # Text color
            text_marker.color.a = 0.6
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0

            marker_array.markers.append(text_marker)

            # make marker for the vertice name
            text_marker2 = copy.deepcopy(text_marker)
            text_marker2.ns = "prm_vert_names" + self.group_name
            text_marker2.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
            text_marker2.text = self.prm.get_vertex_name(conf)

            # Text position
            text_marker2.pose.position.z = text_marker2.pose.position.z - 0.02

            # Text scale
            text_marker2.scale.z = 0.02

            # Text color
            text_marker2.color.a = 0.6
            text_marker2.color.r = 1.0
            text_marker2.color.g = 1.0
            text_marker2.color.b = 1.0

            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker2)

        self.marker_publisher.publish(marker_array)

    def visualize_edges(self, highlight=[], action=visualization_msgs.msg.Marker.ADD):

        marker_array = visualization_msgs.msg.MarkerArray()

        edge_set = self.E_viz_open.copy()
        # rospy.loginfo("edge_set: {}".format(edge_set))
        if len(edge_set) > 0:
            pass
            # rospy.loginfo("edge_set: {}".format(edge_set))

        for edge in edge_set:
            self.E_viz_open.remove(edge)
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = self.prm._base_frame  #_"WAIST"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "prm_lines_" + self.group_name
            # marker.id = self.edges.index(edge)
            marker.id = int(edge.target()) + int(edge.source()) * 10000  # int(edge)

            marker.type = visualization_msgs.msg.Marker.LINE_LIST
            marker.action = action

            # edge_data = self.edges[edge]
            source_node = edge.source()
            destination_node = edge.target()
            for node in [source_node, destination_node]:
                eef_pose = self.prm.get_eef_pose(node)
                pos = eef_pose.pose_stamped[0]
                pt = geometry_msgs.msg.Point()
                pt.x = pos.pose.position.x
                pt.y = pos.pose.position.y
                pt.z = pos.pose.position.z
                marker.points.append(pt)
            # marker.pose.orientation.x = pos.pose.orientation.x
            # marker.pose.orientation.y = pos.pose.orientation.y
            # marker.pose.orientation.z = pos.pose.orientation.z
            # marker.pose.orientation.w = pos.pose.orientation.w
            marker.scale.x = 0.005
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.4
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            # TODO: reactivate highlighting of edges
            # change color for highlighted edges
            # if edge_data[0] in highlight and edge_data[1] in highlight and numpy.abs(highlight.index(edge_data[0]) - highlight.index(edge_data[1])) == 1:
            #     marker.scale.x = 0.007
            #     marker.scale.y = 0.1
            #     marker.scale.z = 0.1
            #     marker.color.a = 0.4
            #     marker.color.r = 0.0
            #     marker.color.g = 1.0
            #     marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def shutdown_hook(self):
        self.timer.shutdown()
        while self.timer.isAlive():
            rospy.sleep(0.5)
            rospy.sleep(0.5)
        self.delete_vis_markers()
        rospy.sleep(1.0)
        rospy.loginfo("Shutdown complete")

    def delete_vis_markers(self):
        self.add_all_nodes_for_viz()
        self.add_all_edges_for_viz()

        self.visualize_prm(action=visualization_msgs.msg.Marker.DELETE)
        self.visualize_edges(action=visualization_msgs.msg.Marker.DELETE)

    def add_all_nodes_for_viz(self):
        self.V_viz_open.clear()
        for vert in self.prm.vertices():
            self.V_viz_open.add(vert)
            # for key in self.V.keys():
            #     self.V_viz_open.add(key)

    def add_all_edges_for_viz(self):
        self.E_viz_open.clear()
        for edge in self.prm.get_edges():
            self.E_viz_open.add(edge)
            # for edge in self.edges:
            #     self.E_viz_open.add(self.edges.index(edge))


if __name__ == "__main__":
    # r = RoadMapVisualizer('prm_r1_arm_2018-05-30 12:27:28.598489')

    # r = RoadMapVisualizer(rm_path="/home/beh2rng/cp_planning_experiments/run_9999/created_in_subprocess_2018-06-12 01:42:35.879541/data_files/roadmaps/prm_r1_arm")

    # r = RoadMapVisualizer("prm_left_arm_2018-08-27 12:09:41.998069")
    # r = RoadMapVisualizer("prm_r2_arm_2018-09-07 11:59:45.851925")
    r = RoadMapVisualizer("prm_r1_arm_2018-09-07 11:59:46.622950")

    print "still here"
