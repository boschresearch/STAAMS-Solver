# Copyright (c) 2018 Robert Bosch GmbH
# All rights reserved.
#
# This source code is licensed under the BSD-3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#
# @author: Jan Behrens

# name of the action template
string name

# the location names to be used for the templated action in the order of appearance in the OVCs
string[] location_names

# a list of OVCs making up the templated action with location names in the schema 'temp_loc_xx', which will be replaced
# by given location names when the OVCs are replicated and filled in.
roadmap_planning_common_msgs/OrderedVisitingConstraint[] ovc

# InterOvcConstraints for the action template
roadmap_planning_common_msgs/InterOVCConstraint[] inter_ovc_ct

