#!/usr/bin/env python

import roslib
roslib.load_manifest('fable_connection')
import rospy

import sys
from fable_connection.node import FableConnectionNode

if __name__ == '__main__':
    fc = FableConnectionNode()



