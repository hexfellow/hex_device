#!/usr/bin/env python
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################

from __future__ import print_function

import threading
import sys
from select import select
# from typing import Dict, Tuple  # Python 2 compatibility

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class TeleopKeyboard:
    """ROS1 Teleop keyboard control interface"""
    
    HELP_MSG = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

    MOVE_BINDINGS = {
        'i': (1, 0, 0, 0),
        'o': (1, 0, 0, -1),
        'j': (0, 0, 0, 1),
        'l': (0, 0, 0, -1),
        'u': (1, 0, 0, 1),
        ',': (-1, 0, 0, 0),
        '.': (-1, 0, 0, 1),
        'm': (-1, 0, 0, -1),
        'O': (1, -1, 0, 0),
        'I': (1, 0, 0, 0),
        'J': (0, 1, 0, 0),
        'L': (0, -1, 0, 0),
        'U': (1, 1, 0, 0),
        '<': (-1, 0, 0, 0),
        '>': (-1, -1, 0, 0),
        'M': (-1, 1, 0, 0),
        't': (0, 0, 1, 0),
        'b': (0, 0, -1, 0),
    }

    SPEED_BINDINGS = {
        'q': (1.1, 1.1),
        'z': (.9, .9),
        'w': (1.1, 1),
        'x': (.9, 1),
        'e': (1, 1.1),
        'c': (1, .9),
    }

    def __init__(self, name="teleop_twist_keyboard"):
        ### name
        self._name = name
        
        ### ros node
        rospy.init_node(name, anonymous=True)

        ### parameters - get ROS parameters
        self.__speed = rospy.get_param("~speed", 0.5)
        self.__turn = rospy.get_param("~turn", 1.0)
        self.__speed_limit = rospy.get_param("~speed_limit", 4.0)
        self.__turn_limit = rospy.get_param("~turn_limit", 1000.0)
        self.__repeat_rate = rospy.get_param("~repeat_rate", 100.0)
        self.__key_timeout = rospy.get_param("~key_timeout", 0.5)
        self.__stamped = rospy.get_param("~stamped", False)
        self.__frame_id = rospy.get_param("~frame_id", '')

        ### setup message type
        self.__twist_msg_type = TwistStamped if self.__stamped else Twist

        ### publish thread
        self.__pub_thread = PublishThread(self.__repeat_rate, self.__twist_msg_type, 
                                        self.__stamped, self.__frame_id)

        ### terminal settings
        self.__settings = self.__save_terminal_settings()

        ### movement state
        self.__x = 0
        self.__y = 0
        self.__z = 0
        self.__th = 0
        self.__status = 0

        ### finish log
        print("#### TeleopKeyboard init: {} ####".format(self._name))

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        self.__pub_thread.stop()
        self.__restore_terminal_settings(self.__settings)

    def __save_terminal_settings(self):
        """Save terminal settings"""
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    def __restore_terminal_settings(self, old_settings):
        """Restore terminal settings"""
        if sys.platform == 'win32':
            return
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def __get_key(self, timeout):
        """Get key input with timeout"""
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            if self.__settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__settings)
        return key

    def __vels_string(self):
        """Generate velocity status string"""
        return "currently:\tspeed {}\tturn {}".format(self.__speed, self.__turn)

    def run(self):
        """Main teleop control loop"""
        try:
            self.__pub_thread.wait_for_subscribers()
            self.__pub_thread.update(self.__x, self.__y, self.__z, self.__th, self.__speed, self.__turn)

            print(self.HELP_MSG)
            print(self.__vels_string())

            while self.ok():
                key = self.__get_key(self.__key_timeout)
                
                if key in self.MOVE_BINDINGS.keys():
                    self.__x = self.MOVE_BINDINGS[key][0]
                    self.__y = self.MOVE_BINDINGS[key][1]
                    self.__z = self.MOVE_BINDINGS[key][2]
                    self.__th = self.MOVE_BINDINGS[key][3]
                    
                elif key in self.SPEED_BINDINGS.keys():
                    self.__speed = min(self.__speed_limit, self.__speed * self.SPEED_BINDINGS[key][0])
                    self.__turn = min(self.__turn_limit, self.__turn * self.SPEED_BINDINGS[key][1])
                    
                    if self.__speed == self.__speed_limit:
                        print("Linear speed limit reached!")
                    if self.__turn == self.__turn_limit:
                        print("Angular speed limit reached!")
                        
                    print(self.__vels_string())
                    if (self.__status == 14):
                        print(self.HELP_MSG)
                    self.__status = (self.__status + 1) % 15
                    
                else:
                    # Skip updating cmd_vel if key timeout and robot already stopped.
                    if key == '' and self.__x == 0 and self.__y == 0 and self.__z == 0 and self.__th == 0:
                        continue
                    self.__x = 0
                    self.__y = 0
                    self.__z = 0
                    self.__th = 0
                    if (key == '\x03'):
                        break

                self.__pub_thread.update(self.__x, self.__y, self.__z, self.__th, self.__speed, self.__turn)

        except Exception as e:
            print("Error in teleop loop: {}".format(e))
        finally:
            self.shutdown()

class PublishThread(threading.Thread):
    def __init__(self, rate, twist_msg_type, stamped, frame_id):
        super(PublishThread, self).__init__()
        self.twist_msg_type = twist_msg_type
        self.stamped = stamped
        self.frame_id = frame_id
        
        self.publisher = rospy.Publisher('cmd_vel', twist_msg_type, queue_size=1)
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = self.twist_msg_type()

        if self.stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = self.frame_id
        else:
            twist = twist_msg
            
        while not self.done:
            if self.stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist_msg)


def main(args=None):
    """Main function to start teleop keyboard control"""
    try:
        teleop = TeleopKeyboard()
        teleop.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print("Error: {}".format(e))


if __name__ == "__main__":
    main()