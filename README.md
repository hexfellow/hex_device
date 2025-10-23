# hex_device

## Overview
This is a ROS package that provides ROS interface for hex robotic devices, including robotic arms and mobile chassis now.

## Prerequisites & Usage
Coming soon...

## Supported Devices

### 1. Arm Device (`arm_trans.py`)
Provides interface for hex robotic arms with optional gripper support.

#### Published Topics
| Topic                         | Msg Type                   | Description                  |
| ----------------------------- | -------------------------- | ---------------------------- |
| `/ws_down`                    | `std_msgs/UInt8MultiArray` | Protobuf messages to device  |
| `/xtopic_arm/joint_states`    | `sensor_msgs/JointState`   | Arm joint states             |
| `/xtopic_arm/gripper_states`  | `sensor_msgs/JointState`   | Gripper states (if equipped) |

#### Subscribed Topics
| Topic                        | Msg Type                        | Description                    |
| ---------------------------- | ------------------------------- | ------------------------------ |
| `/ws_up`                     | `std_msgs/UInt8MultiArray`      | Protobuf messages from device  |
| `/xtopic_arm/joints_cmd`     | `xmsg_comm/XmsgArmJointParamList` | Arm joint control commands   |
| `/xtopic_arm/gripper_cmd`    | `xmsg_comm/XmsgHandCmd`          | Gripper control commands     |

#### Parameters
| Parameter             | Type    | Default | Description                          |
| --------------------- | ------- | ------- | ------------------------------------ |
| `arm_series`         | int     | 0       | Arm series type (e.g., 16 for Archer) |
| `gripper_type`       | int     | 0       | Gripper type (0 = no gripper)        |
| `rate_ros`           | double  | 300.0   | Control loop frequency (Hz)          |
| `joint_config_path`  | string  | None    | Path to joint configuration file     |
| `init_pose_path`     | string  | None    | Path to initial pose configuration   |

---

### 2. Chassis Device (`chassis_trans.py`)
Provides interface for hex mobile chassis with odometry support.

#### Published Topics
| Topic                           | Msg Type                   | Description                  |
| ------------------------------- | -------------------------- | ---------------------------- |
| `/ws_down`                      | `std_msgs/UInt8MultiArray` | Protobuf messages to device  |
| `/xtopic_chassis/motor_states`  | `sensor_msgs/JointState`   | Chassis motor states         |
| `/xtopic_chassis/odom`          | `nav_msgs/Odometry`        | Chassis odometry             |

#### Subscribed Topics
| Topic                        | Msg Type                   | Description                           |
| ---------------------------- | -------------------------- | ------------------------------------- |
| `/ws_up`                     | `std_msgs/UInt8MultiArray` | Protobuf messages from device         |
| `/xtopic_chassis/cmd_vel`    | `geometry_msgs/Twist`      | Velocity commands (simple mode)       |
| `/xtopic_chassis/joint_cmd`  | `sensor_msgs/JointState`   | Joint commands (advanced mode)        |
| `/xtopic_chassis/clear_err`  | `std_msgs/Bool`            | Clear error/parking stop request      |

#### Parameters
| Parameter      | Type    | Default      | Description                                  |
| -------------- | ------- | ------------ | -------------------------------------------- |
| `frame_id`    | string  | "base_link"  | TF frame ID for odometry child frame         |
| `simple_mode` | bool    | true         | Simple mode (cmd_vel) vs advanced (joint_cmd)|
| `rate_ros`    | double  | 100.0        | Control loop frequency (Hz)                  |

---

## Launch Files

### ROS1
- `launch/ros1/arm_bringup.launch` - Launch arm device node
- `launch/ros1/chassis_bringup.launch` - Launch chassis device node

### ROS2
- `launch/ros2/arm_bringup.launch.py` - Launch arm device node
- `launch/ros2/chassis_bringup.launch.py` - Launch chassis device node

## Example Usage

### Launch Arm (ROS1)
```bash
roslaunch hex_device arm_bringup.launch url:=ws://{YOUR_IP}:8439
```

### Launch Arm (ROS2)
```bash
ros2 launch hex_device arm_bringup.launch.py url:=ws://{YOUR_IP}:8439
```

### Launch Chassis (ROS1)
```bash
roslaunch hex_device chassis_bringup.launch url:=ws://{YOUR_IP}:8439
```

### Launch Chassis (ROS2)
```bash
ros2 launch hex_device chassis_bringup.launch.py url:=ws://{YOUR_IP}:8439
```