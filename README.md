# hex_device

## Overview
This is a ROS2 package that provides ROS interface for hex robotic devices, including robotic arms and mobile chassis now.

## Prerequisites & Usage
### 1. Automatically add the hexfellow apt source using our tool
```bash
chmod +x add_hexfellow_apt_source.py && sudo ./add_hexfellow_apt_source.py
```
### 2. Manually add the hexfellow apt source (skip this step if added automatically)
If you don't want to use the script or if it fails, you can manually add the hexfellow apt source.  
If there are old sources, remove them:
```bash
sudo rm -f /etc/apt/sources.list.d/hexfellow.sources  
sudo rm -f /etc/apt/sources.list.d/hexfellow.list  
sudo rm -f /etc/apt/sources.list.d/hexfellow.lists
```
Download the hexfellow GPG key:
```bash
sudo wget -q -O /usr/share/keyrings/hexfellow.gpg.key https://apt.hexfellow.com/conf/hexfellow.gpg.key
```
Add the following text to the /etc/apt/sources.list.d/hexfellow.sources file. If the file doesn't exist, create it:
```
Types: deb
URIs: https://apt.hexfellow.com/hexfellow/
Suites: !!!CHANGE TO YOUR UBUNTU CODENAME!!!
Components: main
Signed-By: /usr/share/keyrings/hexfellow.gpg.key
```
If you don't know your Ubuntu codename, use the lsb_release -a command to check.
For example, if your Ubuntu version is 20.04, modify the text as follows:
```
Types: deb
URIs: https://apt.hexfellow.com/hexfellow/
Suites: focal
Components: main
Signed-By: /usr/share/keyrings/hexfellow.gpg.key
```
### 3. Install xpkg-arm-msgs
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-xpkg-arm-msgs
```
### 4. Download the package
Create a workspace and download the package:
```bash
mkdir -p ~/hex_ws/src
cd ~/hex_ws/src
git clone --recursive https://github.com/hexfellow/hex_bridge.git
git clone --recursive https://github.com/hexfellow/hex_device.git
```
### 5. install protoc
Just choose a suitable version and install it. Here below is an example of installing `protoc-27.1`.
```bash
# For Linux x86_64
wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-x86_64.zip
sudo unzip protoc-27.1-linux-x86_64.zip -d /usr/local
rm protoc-27.1-linux-x86_64.zip  

# For Linux arm64
wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-aarch_64.zip
sudo unzip protoc-27.1-linux-aarch_64.zip -d /usr/local
rm protoc-27.1-linux-aarch_64.zip  

#  Verify installation
protoc --version  # Should show libprotoc 27.1
```
### 6. Install dependencies
```bash
cd ~/hex_ws/src/hex_device/hex_device/
python3 -m pip install -r requirements.txt
```
### 7. Compile Protocol Buffer files
```bash
cd ~/hex_ws/src/hex_device/
./build_proto.sh
```
### 8. Build
```bash
cd ~/hex_ws
colcon build
source install/setup.bash
```
### 9. Usage
Open a new terminal and run:
```bash
ros2 launch hex_bridge hex_bridge.launch.py url:={YOUR_IP}:8439
```
Open another terminal and run:  
If arm:
```bash
ros2 launch hex_device arm_bringup.launch.py
```
If chassis:
```bash
ros2 launch hex_device chassis_bringup.launch.py
```
For chassis, you can also run following commands to control chassis with keyboard:  
Open a new terminal and run:
```bash
ros2 run hex_device chassis_key_control
```

## Supported Devices

### 1. Arm
Provides interface for hex robotic arms with optional gripper support.  
You can remap topics as needed in the launch file.

#### Published Topics
| Topic             | Msg Type                   | Description                  |
| ----------------- | -------------------------- | ---------------------------- |
| `/ws_down`        | `std_msgs/UInt8MultiArray` | Protobuf messages to device  |
| `/joint_states`   | `sensor_msgs/JointState`   | Arm joint states             |
| `/gripper_states` | `sensor_msgs/JointState`   | Gripper states (if equipped) |

#### Subscribed Topics
| Topic          | Msg Type                              | Description                   |
| -------------- | ------------------------------------- | ----------------------------- |
| `/ws_up`       | `std_msgs/UInt8MultiArray`            | Protobuf messages from device |
| `/joints_cmd`  | `xpkg_arm_msgs/XmsgArmJointParamList` | Arm joint control commands    |
| `/gripper_cmd` | `xpkg_arm_msgs/XmsgArmJointParamList` | Gripper control commands      |

#### Parameters
| Parameter           | Type   | Default | Description                           |
| ------------------- | ------ | ------- | ------------------------------------- |
| `arm_series`        | int    | 0       | Arm series type (e.g., 16 for Archer) |
| `gripper_type`      | int    | 0       | Gripper type (0 = no gripper)         |
| `joint_config_path` | string | None    | Path to joint configuration file      |
| `init_pose_path`    | string | None    | Path to initial pose configuration    |

#### Message Type
`xpkg_arm_msgs/XmsgArmJointParamList`:
```
XmsgArmJointParam[] joints
```
`XmsgArmJointParam`:
```
string mode
float64 position
float64 velocity
float64 effort
string extra_param
```
##### Example
There are some example to help you understand how to use this msg.  

Control a joint with the control mode set to position_mode, joint angle of 0.5 radians, velocity of 1.0 radians per second, torque of 1.0 Nm, and no additional parameters:
```bash
ros2 topic pub /joints_cmd xpkg_arm_msgs/XmsgArmJointParamList "joints:
- {mode: 'position_mode', position: 0.5, velocity: 1.0, effort: 1.0, extra_param: ''}"
```
Open the brake when the joint is in position_mode:
```bash
ros2 topic pub /xtopic_arm/joints_cmd xpkg_arm_msgs/XmsgArmJointParamList "joints:
- {mode: 'position_mode', position: 0.5, velocity: 1.0, effort: 1.0, extra_param: '{\"brake\": true}'}'"
```
Control a joint with the control mode set to mit_mode, joint angle of 0.5 radians, velocity of 1.0 radians per second, torque of 1.0 Newtons, and additional parameters: kp=1.0, kd=0.5:
```bash
ros2 topic pub /xtopic_arm/joints_cmd xpkg_arm_msgs/XmsgArmJointParamList "joints:
- {mode:'mit_mode', position: 0.5, velocity: 1.0, effort: 1.0, extra_param: '{\"mit_kp\": 1.0, \"mit_kd\": 0.5}'}'"
```
For multiple joint control, just extend the list of commands.  
Check `pub_xmsg.py` file in hex_device package.

---

### 2. Chassis
Provides interface for hex mobile chassis with odometry support.
You can remap topics as needed in the launch file.

#### Published Topics
| Topic           | Msg Type                   | Description                 |
| --------------- | -------------------------- | --------------------------- |
| `/ws_down`      | `std_msgs/UInt8MultiArray` | Protobuf messages to device |
| `/motor_states` | `sensor_msgs/JointState`   | Chassis motor states        |
| `/odom`         | `nav_msgs/Odometry`        | Chassis odometry            |

#### Subscribed Topics
| Topic        | Msg Type                   | Description                      |
| ------------ | -------------------------- | -------------------------------- |
| `/ws_up`     | `std_msgs/UInt8MultiArray` | Protobuf messages from device    |
| `/cmd_vel`   | `geometry_msgs/Twist`      | Velocity commands (simple mode)  |
| `/joint_cmd` | `sensor_msgs/JointState`   | Joint commands (advanced mode)   |
| `/clear_err` | `std_msgs/Bool`            | Clear error/parking stop request |

#### Parameters
| Parameter     | Type   | Default     | Description                                   |
| ------------- | ------ | ----------- | --------------------------------------------- |
| `frame_id`    | string | "base_link" | TF frame ID for odometry child frame          |
| `simple_mode` | bool   | true        | Simple mode (cmd_vel) vs advanced (joint_cmd) |

---
