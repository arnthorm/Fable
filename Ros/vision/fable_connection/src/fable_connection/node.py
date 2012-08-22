#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Float64

from embedded.messages import SerialTransport
from embedded.helpers import get_usb
from fable_connection.manager import ROSEmbeddedManager
from fable_connection.messages import msgs, field_table, field_conv_table
from fable_connection.messages import messages_collection

from user_tracker.msg import UserRecognized, UserPos, UserAng
from posture_detection.msg import PostureDetected
from std_msgs.msg import String, Int32
from sound_play.libsoundplay import SoundClient
from math import pi

PM_FILTER = 10 

posture_table = {
  'hands_up': 0,
  'hands_down': 1,
  'hands_out': 2,
  'bent_fork': 3,
  'fork': 4,
  'barrels': 5,
  'cross': 6,
  'stop_left': 7,
  'stop_right': 8,
  'surrender': 9,
  'X': 10,
  'Y': 11,
  'hips': 12,
}


conv_value = 1024/pi
def conv_motor_command_pos(value):
  """Convert value in radians to raw (0 - 1023)."""
  value = value + pi/2
  value *= conv_value
  value = round(value)
  if value > 1023:
    value = 1023
  elif value < 0:
    value = 0
  return value

def conv_ang_list_to_raw(value_list):
  """Convert list of radian values to raw (0 - 1023)"""
  def conv(value):
    value *= conv_value
    value = round(value)
    if value > 1023:
      value = 1023
    elif value < 0:
      value = 0
    return value
  for i in xrange(0, len(value_list)):
    value_list[i] = conv(value_list[i])
  return value_list

class FableConnectionNode:
  def __init__(self):
    rospy.init_node('fable_connection', anonymous=True, disable_signals=True)
    self.soundhandle = SoundClient()
    self.manager = None
    self.motor_last_sent = {}
    try:
      self.manager = ROSEmbeddedManager(
        self.setup,
        SerialTransport(
          port=rospy.get_param('~port', get_usb()),
          baud=rospy.get_param('~baudrate', '57600')
        )
      )

      rospy.Subscriber('/posture_detected', PostureDetected, self.posture_detected_cb)

      r = rospy.Rate(5)
      while not rospy.is_shutdown():
        try:
          self.manager.act()
          r.sleep()
        except KeyboardInterrupt:
          self.manager.stop()
          break
    except:
      if self.manager is not None:
        self.manager.stop()
      raise

  def echo_cb(self, message):
    """Callback for echo message."""
    rospy.loginfo('ECHO: ' + str(message.data))

  def debug_cb(self, response):
    """Callback for clear text messages."""
    rospy.loginfo('DEBUG: %s' % str(response))

  def say_cb(self, message):
    """Callback for say message."""
    rospy.loginfo("Say: %s" % message.array)
    if self.soundhandle is not None:
      self.soundhandle.say(message.array)

  def posture_detected_cb(self, data):
    """Callback for ROS /posture_detected message."""
    if self.manager is not None and posture_table.has_key(data.posture):
      self.manager.send(
        msgs.posture_detected,
        user_id=data.user_id, 
        posture=posture_table[data.posture]
      )

  def create_motor_callback(self, i, desc):
    """Create a callback function for ROS /motor_N/command topic."""
    def callback(data):
      d = self.manager.convert_ROS_to_embedded(desc, data)
      d['motor_id'] = i
      pos = d['pos']
      if motor_last_sent[i] is not None and \
         (pos < motor_last_sent[i] - PM_FILTER or \
         self.motor_last_sent[i] + PM_FILTER < pos) or \
         self.motor_last_sent[i] is None:
        self.motor_last_sent[i] = pos
        self.manager.send(msgs.motor_command, **d)
    return callback

  def motor_subscription_cb(self, message):
    """Callback for motor subscription (embedded) message."""
    desc = self.manager.get_message_description(msgs.motor_command)
    self.manager.link(
      msgs.motor_command, Float64,
      field_map={'pos': 'data'},
      ros_emb_conv={'pos': conv_motor_command_pos}
    )
    for i in range(0, message.motor_num):
      self.motor_last_sent[i] = None
      rospy.Subscriber(
        '/motor_%d/command' % i, Float64,
         self.create_motor_callback(i, desc)
      )
    rospy.loginfo("Motor command subscribed! %d motor(s)." % message.motor_num);

  def setup(self, obj):
    """Setup links between ROS messages and embedded messages."""
    obj.register_messages(messages_collection)
    obj.add_field_table(field_table)
    obj.add_field_conv_table(field_conv_table)

    obj.link(msgs.echo, String, '/echo_in', '/echo_out')
    obj.subscribe(msgs.echo, self.echo_cb)
    obj.subscribe(None, self.debug_cb)

    obj.link(msgs.user_detected, Int32, '/user_detected', field_map={'user_id': 'data'})
    obj.link(msgs.user_lost, Int32, '/user_lost', field_map={'user_id': 'data'})
    obj.link(msgs.user_recognized, UserRecognized, '/user_recognized')
    obj.link(msgs.user_pos, UserPos, '/user_pos')
    obj.link(msgs.user_ang, UserAng, '/user_ang',
           ros_emb_conv={'array': conv_ang_list_to_raw})
    obj.subscribe(msgs.say, self.say_cb)
    obj.subscribe(msgs.motor_subscription, self.motor_subscription_cb)

