#!/usr/bin/env python

import roslib
import rospy

from user_tracker.msg import UserPos, UserRecognized
from user_tracker.srv import AddUser, AddUserResponse
from user_tracker.helpers import get_rect_from_point
from recognition.srv import RecognizeFromTopic
from recognition.srv import InsertTopic

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from sound_play.libsoundplay import SoundClient

import tf
import datetime
import math

now = datetime.datetime.now

MAX_HEAD_WIDTH = 0.16
MAX_HEAD_HEIGHT = 0.18
VOICE = 'voice_ked_diphone'
RECOGNIZE_USER_SERVICE = '/recognize/users/from_topic'
#RECOGNIZE_OBJECT_SERVICE = '/recognize/users/from_topic'
RECOGNIZE_OBJECT_SERVICE = '/recognize/objects/from_topic'

class UserRecognition:
  """
  Recognize user from user_tracker with recognition service.

  A manager that handles user recognition, tries to recognize
  new users that user_tracker reports by sending head shots 
  of the user to a recognition service. If the service 
  recognizes this manager publishes to user_recognized topic.
  """
  def __init__(self):
    rospy.init_node('user_recognition', anonymous=True)

    interval_sec = int(rospy.get_param('~interval', 0.3))
    self.interval = datetime.timedelta(0, 0, 1000000*interval_sec)

    self.sub_lost_user = rospy.Subscriber(
      '/user_lost',
      Int32,
      self.callback_user_lost
    )
    self.srv_rec = rospy.Service(
      '/add_user', 
      AddUser,
      self.callback_add_user
    )

    self.srv_rec = rospy.Service(
      '/add_object', 
      AddUser,
      self.callback_add_object
    )

    self.pub_user_recognized = rospy.Publisher(
      '/user_recognized',
      UserRecognized
    )
    self._recognize_user = rospy.ServiceProxy(RECOGNIZE_USER_SERVICE, RecognizeFromTopic)
    self._recognize_object = rospy.ServiceProxy(RECOGNIZE_OBJECT_SERVICE, RecognizeFromTopic)
    self.sub = None # UserPos subscriber handle
    self.users = {}
    self.soundhandle = SoundClient()
    self.wait_for_service()
    rospy.spin()

  def wait_for_service(self):
    """Wait until face recognition service is online."""
    rospy.loginfo("Wait for recognition service ...");
    rospy.wait_for_service(RECOGNIZE_USER_SERVICE)
    rospy.loginfo("Recognition starting ...")
    self.sub = rospy.Subscriber('user_pos', UserPos, self.callback_user_pos, queue_size=1)

  def trigger(self, user_id):
    """
    Trigger recognition for user?

    Returns True if it is time for a recognition service call for user.
    """
    next_trigger = self.userdata(user_id, 'next_trigger')
    start = self.userdata(user_id, 'start')
    now_ = now()
    if next_trigger is not None:
      if now_ < next_trigger:
        return False
    if start is None:
      start = now_
    dt = now_ - start
    min = int(math.floor(dt.total_seconds/60.0))
    if min > 5:
      self.userdata(user_id, 'next_trigger', now_ + self.interval*(360000))
    else:
      self.userdata(user_id, 'next_trigger', now_ + self.interval*(min+1))
    return True

  def callback_user_pos(self, msg):
    """Handler for UserPos message.

    Head position 
    """
    self.userdata(msg.user_id, 'pos', msg)
    username = self.userdata(msg.user_id, 'username')

    # User recognition
    #if not username and self.trigger(msg.user_id):
    if not username:
      head_rect = get_rect_from_point(
        msg.head.position,
        MAX_HEAD_WIDTH,
        MAX_HEAD_HEIGHT
      )

      try:
        start = now()
        self.recognize_user(msg.user_id, head_rect)
        rospy.loginfo("Time: %f", (now()-start).microseconds/1000000.0)
      except rospy.ServiceException, e:
        # If service goes off-line, shutdown and 
        # wait until it comes back on-line.
        rospy.logwarn("Something went wrong with service call. Assuming it went offline.")
        self.sub.unregister()
        self.wait_for_service()
        return
    
    # Object recognition
    if False:
      left_rect = get_rect_from_point(
        msg.left_arm.hand.position,
        MAX_HEAD_WIDTH,
        MAX_HEAD_HEIGHT
      )
      try:
        self.recognize_object(left_rect)
      except:
        rospy.logwarn("Something went wrong with service call. Assuming it went offline.")

        self.sub.unregister()
        self.wait_for_service()
        return

  def recognize_user(self, user_id, rect):
    """
    Recognize user.

    Send a service call to recognition service 
    for a specific user.

    Keyword arguments:
      user_id -- Id of the user to be recognized.
      rect    -- Rectangle of users head position in RGB image.
    """
    response = self._recognize_user(*rect)
    if response.label:
      self.pub_user_recognized.publish(
        UserRecognized(user_id, response.label)
      )
      self.userdata(user_id, 'username', response.label)
      rospy.loginfo('User recognized %s' % response.label)

  def recognize_object(self, rect):
    response = self._recognize_object(*rect)
    if response.label:
      print('Object recognized %s' % response.label)

  def callback_user_lost(self, msg):
    """Remove user data when user is gone."""
    self.remove_userdata(msg.data)

  def callback_add_user(self, request):
    """
    Callback to add user to face recognition database.

    Keyword arguments:
      request.user_id  -- Id of the user to add
      request.username -- Username or label for face.
    """
    user_pos = self.userdata(request.user_id, 'pos')
    success = False
    if user_pos:
      rect = get_rect_from_point(
        user_pos.head.position,
        MAX_HEAD_WIDTH,
        MAX_HEAD_HEIGHT
      )
      try:
        insert_topic = rospy.ServiceProxy('/recognize/users/insert_topic', InsertTopic)
        response = insert_topic(request.username, *rect)
        success = response.success
      except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
        success = False
    return AddUserResponse(success)

  def callback_add_object(self, request):
    user_pos = self.userdata(request.user_id, 'pos')
    success = False
    if user_pos:
      rect = get_rect_from_point(
        user_pos.left_arm.hand.position,
        MAX_HEAD_WIDTH,
        MAX_HEAD_HEIGHT
      )
      try:
        insert_topic = rospy.ServiceProxy('/recognize/objects/insert_topic', InsertTopic)
        response = insert_topic(request.username, *rect)
        success = response.success
      except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
        success = False
    return AddUserResponse(success)

  def remove_userdata(self, user_id):
    """Remove data for user_id."""
    if self.users.has_key(user_id):
      del self.users[user_id]

  def userdata(self, user_id, key, value=None):
    """Gets or sets userdata."""
    if value is not None:
      if not self.users.has_key(user_id):
        self.users[user_id] = {}
      self.users[user_id][key] = value
    else:
      if self.users.has_key(user_id) and \
         self.users[user_id].has_key(key):
        return self.users[user_id][key]
      else:
        return None

if __name__ == '__main__':
  r = UserRecognition()
