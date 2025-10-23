#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2025-07-08
################################################################

import copy
import numpy as np

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from ros_interface import TeleopKeyboard

def main(args=None):
    """Main function to start teleop keyboard control"""
    try:
        teleop = TeleopKeyboard()
        teleop.run()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main() 