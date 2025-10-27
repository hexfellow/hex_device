#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import asyncio
import os
import sys
import time
import signal
import numpy as np

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_device_py import Hands, Arm, public_api_up_pb2, public_api_down_pb2, public_api_types_pb2
from hex_device_py.motor_base import CommandType
from std_msgs.msg import UInt8MultiArray
from xpkg_arm_msgs.msg import XmsgArmJointParamList
from sensor_msgs.msg import JointState


# ========== Device Configuration Mapping ==========

class ArmConfig:
    """Arm configuration: mapping arm type to motor count"""
    def __init__(self):
        self.arm_motor_map = {
            public_api_types_pb2.RobotType.RtArmArcherD6Y: 6,
            public_api_types_pb2.RobotType.RtArmSaberD6X: 6,
            public_api_types_pb2.RobotType.RtArmSaberD7X: 7,
            public_api_types_pb2.RobotType.RtArmSaber750d3Lr3DmDriver: 6,
            public_api_types_pb2.RobotType.RtArmSaber750d4Lr3DmDriver: 7,
            public_api_types_pb2.RobotType.RtArmSaber750h3Lr3DmDriver: 6,
            public_api_types_pb2.RobotType.RtArmSaber750h4Lr3DmDriver: 7,
        }


class GripperConfig:
    """Gripper configuration: mapping gripper type to motor count"""
    def __init__(self):
        self.gripper_motor_map = {
            public_api_types_pb2.HandType.HtInvalid: 0,
            public_api_types_pb2.HandType.HtGp100: 1,
        }

class HexArmApi:
    """
    Hex Arm API
    """

    def __init__(self):
        # 1. Create pure ROS interface
        self.ros_interface = DataInterface(name="xnode_arm")

        # 2. Get parameters
        arm_series = self.ros_interface.get_parameter('arm_series', 0)
        gripper_type = self.ros_interface.get_parameter('gripper_type', 0)
        control_hz = self.ros_interface.get_parameter('rate_ros', 300.0)
        joint_config_path = self.ros_interface.get_parameter('joint_config_path', None)
        init_pose_path = self.ros_interface.get_parameter('init_pose_path', None)

        # 3. Create shared topics (ws_down, ws_up)
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)

        # 4. Create Arm device
        self.arm = None
        self.joint_cmd_sub = None
        self.joint_states_pub = None
        self._is_init = False

        if arm_series != 0 and Arm._supports_robot_type(arm_series):
            arm_config = ArmConfig()
            self.arm = Arm(
                robot_type=arm_series,
                motor_count=arm_config.arm_motor_map[arm_series],
                control_hz=control_hz,
                send_message_callback=self._pub_ws_down
            )
            self.arm._set_robot_type(arm_series)

            # Load configuration using device methods
            # arm joint config is solid for now, this code is not used
            if joint_config_path:
                self.joint_config = self.ros_interface.get_config_from_json(joint_config_path)

            if init_pose_path:
                init_config = self.ros_interface.get_init_pose_config(init_pose_path)
                if init_config:
                    self.init_pose = init_config.get('init_pose', None)
                    self.step_limits = init_config.get('step_limits', None)
                    self.ros_interface.logi(f"Init pose: {self.init_pose}")
                    if self.step_limits:
                        self.ros_interface.logi(f"Step limits: {self.step_limits}")
                else:
                    self.init_pose = None
                    self.step_limits = None
            else:
                self.init_pose = None
                self.step_limits = None

            # Create Arm-specific topics
            self.joint_cmd_sub = self.ros_interface.create_subscription(
                '/xtopic_arm/joints_cmd',
                XmsgArmJointParamList,
                self._joint_cmd_callback,
                10
            )
            self.joint_states_pub = self.ros_interface.create_publisher(
                '/xtopic_arm/joint_states',
                JointState,
                10
            )

            self._is_init = True
        else:
            self._is_init = False
            self.ros_interface.loge(f"Arm series {arm_series} is not supported, initializing failed.")

        # 5. Create Hands device
        self.hands = None
        self.gripper_cmd_sub = None
        self.gripper_states_pub = None

        if gripper_type != 0:
            if Hands._supports_hand_type(gripper_type):
                gripper_config = GripperConfig()
                self.hands = Hands(
                    hand_type=gripper_type,
                    motor_count=gripper_config.gripper_motor_map[gripper_type],
                    control_hz=control_hz,
                    send_message_callback=self._pub_ws_down
                )

                # Create Hands-specific topics
                self.gripper_cmd_sub = self.ros_interface.create_subscription(
                    '/xtopic_arm/gripper_cmd',
                    XmsgArmJointParamList,
                    self._gripper_cmd_callback,
                    10
                )
                self.gripper_states_pub = self.ros_interface.create_publisher(
                    '/xtopic_arm/gripper_states',
                    JointState,
                    10
                )
            else:
                self.ros_interface.loge(f"Unsupported gripper type: {gripper_type}")
        else:
            self.ros_interface.logw("No gripper specified, skipping init gripper")

    async def _pub_ws_down(self, data):
        """Unified ws_down publishing function, shared by all devices"""
        try:
            msg = UInt8MultiArray()
            msg.data = data.SerializeToString()
            self.ros_interface.publish(self.ws_down_pub, msg)
        except Exception:
            pass

    def _ws_up_callback(self, msg):
        """ws_up callback: parse and distribute to corresponding devices"""
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))
        robot_type = api_up.robot_type

        # Distribute to arm
        if self.arm is not None and robot_type == self.arm.robot_type:
            self.arm._update(api_up)

        # Distribute to hands
        if self.hands is not None:
            if hasattr(api_up, 'hand_status') and api_up.HasField('hand_status'):
                self.hands._update_optional_data('hand_status', api_up.hand_status)

    # ========== Arm-specific topic callbacks ==========

    def _joint_cmd_callback(self, msg):
        """Process arm's joint commands"""
        if self.arm is not None and not self._is_init:
            self.ros_interface.process_motor_command(msg, self.arm, 'arm')

    def _publish_joint_states(self):
        """Publish arm's joint states"""
        if self.arm is not None and self.arm._has_new_data:
            motor_status = self.arm.get_simple_motor_status()
            msg = JointState()
            msg.name = [f"joint{i}" for i in range(len(motor_status['pos']))]
            msg.position = motor_status['pos']
            msg.velocity = motor_status['vel']
            msg.effort = motor_status['eff']
            self.ros_interface.publish(self.joint_states_pub, msg)

    # ========== Hands-specific topic callbacks ==========

    def _gripper_cmd_callback(self, msg):
        """Process gripper commands"""
        if self.hands is not None:
            self.ros_interface.process_motor_command(msg, self.hands, 'gripper')

    def _publish_gripper_states(self):
        """Publish gripper states"""
        if self.hands is not None and self.hands._has_new_data:
            motor_status = self.hands.get_simple_motor_status()
            msg = JointState()
            msg.name = [f"gripper{i}" for i in range(len(motor_status['pos']))]
            msg.position = motor_status['pos']
            msg.velocity = motor_status['vel']
            msg.effort = motor_status['eff']
            self.ros_interface.publish(self.gripper_states_pub, msg)

async def _run_with_cancellation(coro, stop_event, loop):
    """Wrapper to run coroutine and check for stop event"""
    task = asyncio.create_task(coro)
    try:
        # Run task but check stop event periodically
        while not task.done():
            if stop_event.is_set():
                task.cancel()
                break
            await asyncio.sleep(0.01)
        await task
    except asyncio.CancelledError:
        # Task was cancelled, this is expected
        pass
    except Exception as e:
        print(f"Task exception: {e}")


def run_async_in_thread(coro, stop_event):
    """Helper function to run async coroutine in a thread with a new event loop"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_run_with_cancellation(coro, stop_event, loop))
    except Exception as e:
        print(f"Async thread exception: {e}")
    finally:
        # Cancel all remaining tasks
        try:
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
            # Wait for all tasks to complete cancellation
            if pending:
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        except Exception:
            pass
        finally:
            loop.close()

def signal_handler(signum, frame, stop_event, arm_thread, hands_thread, api):
    """Custom signal handler for graceful shutdown"""
    # stop arm
    print("\n[Ctrl-C] Received shutdown signal")
    api.arm.stop()
    time.sleep(0.5)

    # Signal threads to stop
    print("[Shutdown] Stopping async threads...")
    stop_event.set()
    
    # Wait for threads to finish with timeout
    if arm_thread and arm_thread.is_alive():
        arm_thread.join(timeout=2.0)
        if arm_thread.is_alive():
            print("[Warning] Arm thread did not stop gracefully")
    
    if hands_thread and hands_thread.is_alive():
        hands_thread.join(timeout=2.0)
        if hands_thread.is_alive():
            print("[Warning] Hands thread did not stop gracefully")
    
    print("[Shutdown] Shutting down ROS interface...")
    api.ros_interface.shutdown()
    print("[Shutdown] Complete")
    sys.exit(0)


# ========== Main Function ==========

def main():
    """
    Main function
    Manages arm and gripper devices using new architecture
    """
    api = HexArmApi()

    # Create stop event for graceful shutdown
    stop_event = threading.Event()

    # Start arm thread if arm exists
    arm_thread = None
    if api.arm is not None:
        arm_thread = threading.Thread(
            target=run_async_in_thread,
            args=(api.arm._periodic(), stop_event)
        )
        arm_thread.daemon = True
        arm_thread.start()
    else:
        api.ros_interface.loge("Arm not initialized, skipping arm thread.")
        return

    # Start hands thread if hands exists
    hands_thread = None
    if api.hands is not None:
        hands_thread = threading.Thread(
            target=run_async_in_thread,
            args=(api.hands._periodic(), stop_event)
        )
        hands_thread.daemon = True
        hands_thread.start()

    # wait thread to start
    time.sleep(0.2)

    signal.signal(signal.SIGINT, lambda signum, frame: signal_handler(signum, frame, stop_event, arm_thread, hands_thread, api))
    signal.signal(signal.SIGTERM, lambda signum, frame: signal_handler(signum, frame, stop_event, arm_thread, hands_thread, api))

    while api.ros_interface.ok():
        # Initial pose control
        if api.arm is not None and api._is_init:
            if api.init_pose is not None and isinstance(api.init_pose, list):
                # Get current position
                current_pos = np.array(api.arm.get_motor_positions())
                target_pos = np.array(api.init_pose)
                err = target_pos - current_pos

                # Send command to arm
                if api.step_limits is not None:
                    step_limits = np.array(api.step_limits)
                    err = np.clip(err, -step_limits, step_limits)
                next_pos = current_pos + err
                api.arm.motor_command(CommandType.POSITION, next_pos.tolist())

                # Check if init pose is reached (within tolerance)
                if np.allclose(current_pos, target_pos, atol=0.01):
                    api.ros_interface.logi("Init pose reached.")
                    api._is_init = False
            else:
                api.ros_interface.logi("Init pose is not set or is not a list, skipping init.")
                api._is_init = False

        # Publish states
        api._publish_joint_states()
        api._publish_gripper_states()

        # Sleep
        api.ros_interface.sleep()


if __name__ == '__main__':
    main()
