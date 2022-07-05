#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# TODO: 外部でのパス
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))
print(sys.path)
from mqtt_bridge.app import mqtt_bridge_node


try:
    mqtt_bridge_node()
except rospy.ROSInterruptException:
    pass
