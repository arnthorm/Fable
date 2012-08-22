
import rospy

from user_tracker.helpers import get_joint_angles_from_msg, get_attr

import cPickle as pickle
import math
import datetime
import os

now = datetime.datetime.now
dt = datetime.timedelta(0, 0, 500000)


class PostureManager:
  def __init__(self, database_path='db.p'):

    self._angle_cache = None
    self.database_path = database_path
    self._load_database()
    self.detected = None
    self.timeout = None
    self.threshold_lower = 0.55
    self.threshold_upper = 0.7

  def _load_database(self):
    """
    Load database from file into memory.

    The database is a dictionary, structured as following:
    {
      'posture_name1': {'angle1': 3.14, 'angle2': 1.5, ..., 'angleN': 1.6},
      'posture_name2': {'angle1': 1.5, 'angle2': 3.14, ..., 'angleN': 0},
      ...
      'posture_nameN': {'angle1': 1.5, 'angle2': 3.14, ..., 'angleN': 0},
    }
    """
    if os.path.exists(self.database_path):
      self.database = pickle.load(open(self.database_path, "rb"))
    else:
      self.database = {}
    rospy.loginfo("Loaded %d postures" % len(self.database.keys()))

  def _save_database(self):
    """Save current database to file (using Pickle)."""
    pickle.dump(self.database, open(self.database_path, "wb"))

  def _calc_dist(self, posture, angles):
    """
    Calculates the Euclidean distance for postures.

    Keyword arguments:
      posture -- Predefined posture to compare with.
      angles  -- Current joint angles of a user.
      posture and angles are dictionaries of joint names
      and their angles.
    """
    value = 0
    for key in posture:
      value += (posture[key] - angles[key])**2
    return math.sqrt(value)

  def _find_min(self, items):
    """
    Find the minimum euclidean distance in a list.
    """
    min = items[0][1]
    min_item = items[0]
    for item in items:
      if item[1] < min:
        min_item = item
        min = item[1]
    return min_item

  def _convert_user_ang(self, user_ang):
    """Convert fields of UserAng message into dictionary."""
    if not self._angle_cache:
      self._angle_cache = get_joint_angles_from_msg(user_ang)

    d = {}
    for angle in self._angle_cache:
      d[angle] = get_attr(user_ang, angle).angle
    return d
  
  def detect_posture(self, user_angles):
    """
    Compares given user angles to posture database and returns name of
    posture if detected.

    Keyword arguments:
      user_angles -- UserAng message with angles of the users joints.
    """
    if self.database:
      dists = []
      user_angles = self._convert_user_ang(user_angles)
      for posture in self.database:
        dists.append(
          [
            posture, 
            self._calc_dist(self.database[posture], user_angles)
          ]
        )
      min_posture = self._find_min(dists)
      return min_posture
    return None

  def posture_event(self, user_angles):
    min_posture = self.detect_posture(user_angles)
    if min_posture is None:
      return None

    # TODO: The states must be dependant on user id!
    if self.detected:
      if self.timeout < now() or \
         min_posture[0] != self.detected and min_posture[1] < self.threshold_lower:
        self.detected = None
      elif min_posture[1] < self.threshold_upper:
        self.timeout = now() + dt

    if not self.detected and min_posture[1] < self.threshold_lower:
      self.detected = min_posture[0]
      self.timeout = now() + dt
      return min_posture[0]

    return None


  def add_posture(self, posture_name, user_angles, force=True):
    """
    Adds a user angles to posture database
    
    Keyword arguments:
      posture_name -- Name of posture 
      user_angles  -- UserAng message with angles of the users joints. 
      force        -- Override if posture already exists.
    """
    if posture_name in self.database and force or \
       posture_name not in self.database:
      self.database.update({posture_name: self._convert_user_ang(user_angles)})
      self._save_database()
      return True
    else:
      return False

  def remove_posture(self, posture_name):
    """Remove posture from database."""
    if posture_name in self.database:
      del self.database[posture_name]
      self._save_database()

  def remove_angles_from_posture(self, posture_name, joint_angles):
    """
    Remove specific angles from posture in database.

    Keyword arguments:
      posture_name -- Posture name, to remove from.
      joint_angles -- List containing names of joint angles.
    """
    if posture_name in self.database:
      for angle in joint_angles:
        if angle in self.database[posture_name]:
          del self.database[posture_name][angle]
      self._save_database()

