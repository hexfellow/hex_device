#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import rospy
import rospkg
from typing import Callable
from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):
    """
    ROS1 interface implementation
    Only provides ROS1 atomic operations, does not preset any topics or callbacks
    """

    def __init__(self, name: str):
        super(DataInterface, self).__init__(name=name)

        # Initialize ROS1 node
        rospy.init_node(self._name, anonymous=True)

        # Initialize rate control
        self.ros_rate = rospy.get_param('~rate_ros', 300.0)
        self._rate = rospy.Rate(self.ros_rate)

        ### ROS1 does not need to use spin, and can also use sleep() to process callback
        # Start spin thread
        # self.__spin_thread = threading.Thread(target=self.__spin)
        # self.__spin_thread.start()

    # ========== Topic management (generic) ==========

    def create_publisher(self, topic_name: str, msg_type: type, queue_size: int = 10):
        """
        Create publisher

        Args:
            topic_name: Topic name
            msg_type: Message type
            queue_size: Queue size

        Returns:
            Publisher object
        """
        try:
            return rospy.Publisher(topic_name, msg_type, queue_size=queue_size)
        except Exception as e:
            self.loge(f"Failed to create publisher for {topic_name}: {e}")
            return None

    def create_subscription(self, topic_name: str, msg_type: type,
                           callback: Callable, queue_size: int = 10):
        """
        Create subscription

        Args:
            topic_name: Topic name
            msg_type: Message type
            callback: Callback function (defined externally)
            queue_size: Queue size

        Returns:
            Subscription object
        """
        try:
            return rospy.Subscriber(topic_name, msg_type, callback, queue_size=queue_size)
        except Exception as e:
            self.loge(f"Failed to create subscription for {topic_name}: {e}")
            return None

    def publish(self, publisher, msg):
        """
        Publish message

        Args:
            publisher: Publisher object (returned by create_publisher)
            msg: Message object
        """
        try:
            publisher.publish(msg)
        except Exception:
            # Silently ignore publish errors during shutdown
            pass

    # ========== Timer ==========

    def create_timer(self, interval_sec: float, callback: Callable[[], None]):
        """
        Create timer

        Args:
            interval_sec: Timer interval (seconds)
            callback: Callback function (no parameters, defined externally)

        Returns:
            Timer object

        Note:
            ROS1 timer callback requires TimerEvent parameter,
            but we wrap it here to provide unified interface
        """
        try:
            # Wrap callback to hide event parameter
            def wrapped_callback(event):
                callback()

            return rospy.Timer(rospy.Duration(interval_sec), wrapped_callback)
        except Exception as e:
            self.loge(f"Failed to create timer: {e}")
            return None

    # ========== Parameter server ==========

    def get_parameter(self, name: str, default=None):
        """
        Get parameter

        Args:
            name: Parameter name
            default: Default value

        Returns:
            Parameter value
        """
        try:
            return rospy.get_param(f'~{name}', default)
        except Exception:
            return default

    def set_parameter(self, name: str, value):
        """
        Set parameter

        Args:
            name: Parameter name
            value: Parameter value
        """
        try:
            rospy.set_param(name, value)
        except Exception as e:
            self.loge(f"Failed to set parameter {name}: {e}")

    # ========== Rate control ==========

    def set_rate(self, hz: float):
        """
        Set rate controller frequency

        Args:
            hz: Frequency (Hz)
        """
        try:
            self._rate = rospy.Rate(hz)
        except Exception as e:
            self.loge(f"Failed to set rate: {e}")

    def sleep(self):
        """Sleep according to rate"""
        try:
            self._rate.sleep()
        except Exception:
            pass

    # ========== Lifecycle ==========

    def ok(self) -> bool:
        """Check if ROS is running normally"""
        return not rospy.is_shutdown()

    def shutdown(self):
        """Shutdown ROS node"""
        try:
            # Signal ROS shutdown
            if not rospy.is_shutdown():
                rospy.signal_shutdown("Normal shutdown")
                
            ### ROS1 does not need to use spin, and can also use sleep() to process callback
            # Wait for spin thread to finish
            # if self.__spin_thread and self.__spin_thread.is_alive():
            #     self.__spin_thread.join(timeout=1.0)
        except Exception:
            pass

    # ========== Logging ==========

    def logd(self, msg: str, *args, **kwargs):
        """Output debug log"""
        try:
            rospy.logdebug(msg, *args, **kwargs)
        except Exception:
            pass

    def logi(self, msg: str, *args, **kwargs):
        """Output info log"""
        try:
            rospy.loginfo(msg, *args, **kwargs)
        except Exception:
            pass

    def logw(self, msg: str, *args, **kwargs):
        """Output warn log"""
        try:
            rospy.logwarn(msg, *args, **kwargs)
        except Exception:
            pass

    def loge(self, msg: str, *args, **kwargs):
        """Output error log"""
        try:
            rospy.logerr(msg, *args, **kwargs)
        except Exception:
            pass

    def logf(self, msg: str, *args, **kwargs):
        """Output fatal log"""
        try:
            rospy.logfatal(msg, *args, **kwargs)
        except Exception:
            pass

    # ========== Tools ==========

    def get_pkg_share_path(self, package_name: str) -> str:
        """Get ROS package shared directory path"""
        try:
            rospack = rospkg.RosPack()
            return rospack.get_path(package_name)
        except rospkg.ResourceNotFound:
            self.loge(f"Package '{package_name}' not found.")
            return ""
        except Exception as e:
            self.loge(f"An error occurred while getting the path for package '{package_name}': {e}")
            return ""
        
    def get_timestamp(self):
        return rospy.Time.now()

    # ========== Internal methods ==========

    def __spin(self):
        """Spin ROS1 in separate thread"""
        try:
            rospy.spin()
        except Exception:
            # Ignore exceptions during shutdown (ROSInterruptException)
            pass