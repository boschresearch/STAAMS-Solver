#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import rospy
from roadmap_tools.service_proxies import ServiceProxies



if __name__ == "__main__":
    ServiceProxies.reload_scene_graph_client('3')