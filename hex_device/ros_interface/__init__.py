#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os

ROS_VERSION = os.environ.get('ROS_VERSION')
if ROS_VERSION == '1':
    from .ros1_interface import DataInterface as DataInterface
    from .ros1_chassis_key_control import TeleopKeyboard as TeleopKeyboard
elif ROS_VERSION == '2':
    from .ros2_interface import DataInterface as DataInterface
    from .ros2_chassis_key_control import TeleopKeyboard as TeleopKeyboard
else:
    raise ValueError("ROS_VERSION is not set")


__all__ = [
    "DataInterface",
    "TeleopKeyboard",
]