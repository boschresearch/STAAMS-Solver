#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from geometry_msgs.msg import TransformStamped
from roadmap_planner_tools.planner_input_manager import PlannerInputManager
from roadmap_planning_common_msgs.msg import OrderedVisitingConstraint, StringList, ConstraintType
from roadmap_planning_common_msgs.srv import AddObjectRequest
from rospy_message_converter import json_message_converter


def get_pose_dict():
    # type: () -> dict[str, PoseStamped]
    PosesDict = {}

    s =  '{"object_type": "gluepoint", "object_name": "loc_8", "pose": {"header": {"stamp": {"secs": 1527494081, "nsecs": 245750904}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.0673123623248813, "x": -0.11819957737525943, "z": -0.000529293203694906}, "orientation": {"y": -0.09199954920780186, "x": -0.02204239273911617, "z": -0.9826223619036331, "w": -0.15969818331722338}}}}' + '\n'
    s += '{"object_type": "gluepoint", "object_name": "loc_6", "pose": {"header": {"stamp": {"secs": 1527494075, "nsecs": 379102230}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.11369306929267338, "x": -0.1261289291850433, "z": 0.0007951176021754491}, "orientation": {"y": 0.07187094200286825, "x": -0.061023926496261725, "z": 0.9873831660085665, "w": 0.12722079850990872}}}}' + '\n'
    s += '{"object_type": "gluepoint", "object_name": "loc_7", "pose": {"header": {"stamp": {"secs": 1527494078, "nsecs": 595736504}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.1149274695384531, "x": 0.06208364635543662, "z": -0.005476238253788906}, "orientation": {"y": 0.1316436407714954, "x": 0.019739166149056388, "z": 0.9750055991865761, "w": 0.17788872566576838}}}}' + '\n'
    s += '{"object_type": "gluepoint", "object_name": "loc_4", "pose": {"header": {"stamp": {"secs": 1527494065, "nsecs": 979056120}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.06856865760540547, "x": 0.4478018813158141, "z": -0.000679487573898074}, "orientation": {"y": -0.050516132689598016, "x": 0.014163494691613031, "z": -0.878984408924756, "w": -0.47395561461323}}}}' + '\n'
    s += '{"object_type": "gluepoint", "object_name": "loc_5", "pose": {"header": {"stamp": {"secs": 1527494071, "nsecs": 795750141}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.06760627153697468, "x": 0.06349269911330815, "z": -0.0007470379806025116}, "orientation": {"y": -0.010168248374561623, "x": 0.04411559477008324, "z": -0.9325496611657705, "w": -0.3581920580954878}}}}' + '\n'
    s += '{"object_type": "gluepoint", "object_name": "loc_2", "pose": {"header": {"stamp": {"secs": 1527494059, "nsecs": 112413883}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.06838950251450462, "x": 0.6409328063798745, "z": 0.00015782094835932174}, "orientation": {"y": 0.05237392498219545, "x": 0.02846261965189043, "z": 0.8403338301717435, "w": 0.53878187157086}}}}' + '\n'
    s += '{"object_type": "gluepoint", "object_name": "loc_3", "pose": {"header": {"stamp": {"secs": 1527494063, "nsecs": 79089880}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.11665509017783073, "x": 0.6388356663857032, "z": 0.001613388200793883}, "orientation": {"y": -0.10153267056716704, "x": -0.0370089029955912, "z": -0.9240133314356973, "w": -0.3667708020489993}}}}' + '\n'
    s += '{"object_type": "gluepoint", "object_name": "loc_1", "pose": {"header": {"stamp": {"secs": 1527494055, "nsecs": 529085398}, "frame_id": "board", "seq": 0}, "pose": {"position": {"y": 0.11632455207600452, "x": 0.4453907194544092, "z": 0.0016855318552673815}, "orientation": {"y": -0.11786933993294174, "x": -0.08813291134398896, "z": -0.9653377546033448, "w": -0.21555145135020032}}}}' + '\n'

    json_strs = s.readlines()

    for json in json_strs:
        if json == "\n":
            continue
        req = json_message_converter.convert_json_to_ros_message('roadmap_planning_common_msgs/AddObjectRequest', json) # type: AddObjectRequest
        PosesDict[req.object_name] = req.pose


    return PosesDict

if __name__ == "__main__":
    pi_manager = PlannerInputManager()

    trans = TransformStamped()
    trans.child_frame_id = 'board'
    trans.header.frame_id = 'world'
    board_quat = [-0.6646584989424609, 0.7469166744613165, 0.009387090228191897, -0.016013860629187193]
    board_trans = [0.6, 0.3, 0.02]
    trans.transform.translation.x = board_trans[0]
    trans.transform.translation.y = board_trans[1]
    trans.transform.translation.z = board_trans[2]
    trans.transform.rotation.x = board_quat[0]
    trans.transform.rotation.y = board_quat[1]
    trans.transform.rotation.z = board_quat[2]
    trans.transform.rotation.w = board_quat[3]

    pi_manager.add_frame(transform=trans)

    PosesDict = get_pose_dict()
    pi_manager.add_loc(PosesDict.values(), PosesDict.keys(), len(PosesDict) * ["gluepoint"])

    myOVC = OrderedVisitingConstraint()
    myOVC.name = 'ovc_1'
    loc_names_1 = StringList()
    loc_names_1.values.append('loc_1')
    myOVC.location_names.append(loc_names_1)
    pi_manager.add_ovc([myOVC])

    myOVC_2 = OrderedVisitingConstraint()
    myOVC_2.name = 'ovc_2'
    loc_names_1 = StringList()
    loc_names_1.values.append('loc_2')
    myOVC_2.location_names.append(loc_names_1)
    pi_manager.add_ovc([myOVC])

    # ct = Constraints[0]
    pi_manager.add_ovc_ct(constraint_type=ConstraintType.StartsAfterEnd, first_ovcs=['ovc_1'], second_ovcs=['ovc_2'])

    pi_manager.write_planner_input_file('test')