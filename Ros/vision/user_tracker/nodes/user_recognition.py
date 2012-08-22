#!/usr/bin/env python

import roslib
roslib.load_manifest('user_tracker')
import rospy

from user_tracker.user_recognition import UserRecognition

if __name__ == '__main__':
  p = UserRecognition()
