# hex_device

## Overview
This is a ROS2 package that provides ROS interface for hex robotic devices, including robotic arms and mobile chassis now.

## Prerequisites & Usage
Coming soon...

## Supported Devices

### 1. Arm Device (`arm_trans.py`)
Provides interface for hex robotic arms with optional gripper support.
You can remap topics as needed in the launch file.

#### Published Topics
| Topic                        | Msg Type                   | Description                  |
| ---------------------------- | -------------------------- | ---------------------------- |
| `/ws_down`                   | `std_msgs/UInt8MultiArray` | Protobuf messages to device  |
| `/xtopic_arm/joint_states`   | `sensor_msgs/JointState`   | Arm joint states             |
| `/xtopic_arm/gripper_states` | `sensor_msgs/JointState`   | Gripper states (if equipped) |

#### Subscribed Topics
| Topic                     | Msg Type                          | Description                   |
| ------------------------- | --------------------------------- | ----------------------------- |
| `/ws_up`                  | `std_msgs/UInt8MultiArray`        | Protobuf messages from device |
| `/xtopic_arm/joints_cmd`  | `xmsg_comm/XmsgArmJointParamList` | Arm joint control commands    |
| `/xtopic_arm/gripper_cmd` | `xmsg_comm/XmsgHandCmd`           | Gripper control commands      |

#### Parameters
| Parameter           | Type   | Default | Description                           |
| ------------------- | ------ | ------- | ------------------------------------- |
| `arm_series`        | int    | 0       | Arm series type (e.g., 16 for Archer) |
| `gripper_type`      | int    | 0       | Gripper type (0 = no gripper)         |
| `rate_ros`          | double | 300.0   | Control loop frequency (Hz)           |
| `joint_config_path` | string | None    | Path to joint configuration file      |
| `init_pose_path`    | string | None    | Path to initial pose configuration    |

---

### 2. Chassis Device (`chassis_trans.py`)
Provides interface for hex mobile chassis with odometry support.
You can remap topics as needed in the launch file.

#### Published Topics
| Topic                          | Msg Type                   | Description                 |
| ------------------------------ | -------------------------- | --------------------------- |
| `/ws_down`                     | `std_msgs/UInt8MultiArray` | Protobuf messages to device |
| `/xtopic_chassis/motor_states` | `sensor_msgs/JointState`   | Chassis motor states        |
| `/xtopic_chassis/odom`         | `nav_msgs/Odometry`        | Chassis odometry            |

#### Subscribed Topics
| Topic                       | Msg Type                   | Description                      |
| --------------------------- | -------------------------- | -------------------------------- |
| `/ws_up`                    | `std_msgs/UInt8MultiArray` | Protobuf messages from device    |
| `/cmd_vel`                  | `geometry_msgs/Twist`      | Velocity commands (simple mode)  |
| `/xtopic_chassis/joint_cmd` | `sensor_msgs/JointState`   | Joint commands (advanced mode)   |
| `/xtopic_chassis/clear_err` | `std_msgs/Bool`            | Clear error/parking stop request |

#### Parameters
| Parameter     | Type   | Default     | Description                                   |
| ------------- | ------ | ----------- | --------------------------------------------- |
| `frame_id`    | string | "base_link" | TF frame ID for odometry child frame          |
| `simple_mode` | bool   | true        | Simple mode (cmd_vel) vs advanced (joint_cmd) |
| `rate_ros`    | double | 100.0       | Control loop frequency (Hz)                   |

---

## Example Usage

### Launch Arm
```bash
ros2 launch hex_device arm_bringup.launch.py url:={YOUR_IP}:8439 enable_bridge:=true
```

### Launch Chassis
```bash
ros2 launch hex_device chassis_bringup.launch.py url:={YOUR_IP}:8439 enable_bridge:=true
```