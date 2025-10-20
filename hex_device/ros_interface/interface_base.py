#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
ROS interface abstract base class
Only provides pure ROS capabilities, does not preset any topics or callbacks
"""

from abc import ABC, abstractmethod
from typing import Callable, Any


class InterfaceBase(ABC):
    """
    ROS interface abstract base class
    Only provides ROS atomic operation capabilities, does not preset any topics or callbacks
    """

    def __init__(self, name: str):
        self._name = name

    # ========== Topic management (generic) ==========

    @abstractmethod
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
        pass

    @abstractmethod
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
        pass

    @abstractmethod
    def publish(self, publisher, msg):
        """
        Publish message

        Args:
            publisher: Publisher object (returned by create_publisher)
            msg: Message object
        """
        pass

    # ========== Timer ==========

    @abstractmethod
    def create_timer(self, interval_sec: float, callback: Callable[[], None]):
        """
        Create timer

        Args:
            interval_sec: Timer interval (seconds)
            callback: Callback function (no parameters, defined externally)

        Returns:
            Timer object

        Note:
            - User-provided callback does not need to receive any parameters
            - ROS1/ROS2 internally handle differences
        """
        pass

    # ========== Parameter server ==========

    @abstractmethod
    def get_parameter(self, name: str, default=None):
        """
        Get parameter

        Args:
            name: Parameter name
            default: Default value

        Returns:
            Parameter value
        """
        pass

    @abstractmethod
    def set_parameter(self, name: str, value):
        """
        Set parameter

        Args:
            name: Parameter name
            value: Parameter value
        """
        pass

    # ========== Rate control ==========

    @abstractmethod
    def set_rate(self, hz: float):
        """
        Set rate controller frequency

        Args:
            hz: Frequency (Hz)
        """
        pass

    @abstractmethod
    def sleep(self):
        """Sleep according to rate"""
        pass

    # ========== Lifecycle ==========

    @abstractmethod
    def ok(self) -> bool:
        """Check if ROS is running normally"""
        pass

    @abstractmethod
    def shutdown(self):
        """Shutdown ROS node"""
        pass

    # ========== Logging ==========

    @abstractmethod
    def logd(self, msg: str, *args, **kwargs):
        """Output debug log"""
        pass

    @abstractmethod
    def logi(self, msg: str, *args, **kwargs):
        """Output info log"""
        pass

    @abstractmethod
    def logw(self, msg: str, *args, **kwargs):
        """Output warn log"""
        pass

    @abstractmethod
    def loge(self, msg: str, *args, **kwargs):
        """Output error log"""
        pass

    @abstractmethod
    def logf(self, msg: str, *args, **kwargs):
        """Output fatal log"""
        pass

    # ========== Tools ==========

    @abstractmethod
    def get_pkg_share_path(self, package_name: str) -> str:
        """Get ROS package shared directory path"""
        pass
