#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
import os
import sys
import time
script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)
from ros_interface import DataInterface
from xpkg_arm_msgs.msg import XmsgArmJointParam, XmsgArmJointParamList

class XmsgInterface:
    def __init__(self, node_name: str):
        self.data_interface = DataInterface(node_name)
        self._joints_cmd_pub = self.data_interface.create_publisher("/joints_cmd", XmsgArmJointParamList)

    def pub_joints_cmd(self):
        msg = XmsgArmJointParamList(
            joints=[
                XmsgArmJointParam(mode="torque_mode", position=-0.3, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="torque_mode", position=-1.48, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="torque_mode", position=2.86, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="torque_mode", position=0.0, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="torque_mode", position=0.0, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 39.0, \"mit_kd\": 0.8}"),
                XmsgArmJointParam(mode="torque_mode", position=0.0, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 39.0, \"mit_kd\": 0.8}"),
            ]
        )
        self.data_interface.publish(self._joints_cmd_pub, msg)

def main():
    xmsg_interface = XmsgInterface("xmsg_pub")
    control_hz = 100.0
    try:
        cycle_time = 1000.0 / control_hz
        while True:
            start_time = time.perf_counter()
            end_time = start_time + cycle_time / 1000
            xmsg_interface.pub_joints_cmd()
            now = time.perf_counter()
            sleep_time = end_time - now
            time.sleep(sleep_time) 
    except KeyboardInterrupt:
        xmsg_interface.data_interface.shutdown() 
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()