#!/usr/bin/env python

import roslib
roslib.load_manifest('posture_detection')
import rospy

import sys
from posture_detection.node import PostureNode

USAGE = """
Usage: rosrun posture.py [add|remove] [...]
  Commands:
    add: Add new posture to the database
      rosrun posture.py add posture_name [arms|legs]
    remove: Remove existing posture from the database
      rosrun posture.py remove posture_name
"""

if __name__ == '__main__':
  if len(sys.argv) > 1:
    if sys.argv[1] == 'help':
      print USAGE 
    args = []
    if len(sys.argv) > 2:
      args = sys.argv[2:]
    p = PostureNode(command=sys.argv[1], args=args)
  else:
    p = PostureNode()

