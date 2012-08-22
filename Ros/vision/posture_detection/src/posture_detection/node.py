
import rospy
import roslib

from posture_detection.manager import PostureManager
from posture_detection.msg import PostureDetected
from user_tracker.msg import UserAng

all = ['left_arm/elbow_pitch', 'left_arm/shoulder_pitch',
          'left_arm/shoulder_roll', 'left_arm/shoulder_yaw',
          'left_leg/hip_pitch', 'left_leg/hip_roll', 'left_leg/hip_yaw',
          'left_leg/knee_pitch', 'neck', 'right_arm/elbow_pitch',
          'right_arm/shoulder_pitch', 'right_arm/shoulder_roll',
          'right_arm/shoulder_yaw', 'right_leg/hip_pitch', 'right_leg/hip_roll',
          'right_leg/hip_yaw', 'right_leg/knee_pitch', 'torso']

arms = ['left_arm/shoulder_pitch', 'left_arm/shoulder_yaw',
        'left_arm/sholder_roll', 'left_arm/elbow_pitch',
        'right_arm/shoulder_pitch', 'right_arm/shoulder_yaw',
        'right_arm/sholder_roll', 'right_arm/elbow_pitch']

legs = ['left_leg/hip_pitch', 'left_leg/hip_roll', 
        'left_leg/hip_yaw','left_leg/knee_pitch', 
        'right_leg/hip_pitch', 'right_leg/hip_roll',
        'right_leg/hip_yaw', 'right_leg/knee_pitch']

body = ['body',]


class PostureNode:
  def __init__(self, command=None, args=[]):
    self.manager = PostureManager('%s/db.p' %
                                  roslib.packages.get_pkg_dir('posture_detection'))
    self.subscriptions = []
    self.command = command
    self.args = args

    rospy.init_node('posture', anonymous=True)

    if command == 'add':
      self.subscriptions.append(
        rospy.Subscriber('user_ang', UserAng, self.snapshot_callback)
      )
    elif command == 'remove':
      self.manager.remove_posture(self.args[0])
      rospy.signal_shutdown('')
    elif command == 'list':
      for key in self.manager.database:
        print key
      rospy.signal_shutdown('')
    else:
      rospy.Subscriber('user_ang', UserAng, self.userAng_callback)

    self.pub = rospy.Publisher('posture_detected', PostureDetected)
    rospy.spin()

  def _remove_subscriptions(self):
    for sub in self.subscriptions:
      sub.unregister()

  def snapshot_callback(self, message):
    rospy.loginfo('Snapshot')
    self.manager.add_posture(self.args[0], message)
    self.manager.remove_angles_from_posture(self.args[0], body)
    if len(self.args) > 1:
      to_remove = legs if self.args[1] == 'arms' else arms
      self.manager.remove_angles_from_posture(self.args[0], to_remove)
    self._remove_subscriptions()
    rospy.signal_shutdown('')

  def userAng_callback(self, message):
    posture_detected = self.manager.posture_event(message)
    if posture_detected:
      rospy.loginfo("Posture detected: %s", posture_detected)
      p = PostureDetected()
      p.user_id = message.user_id
      p.posture = posture_detected
      self.pub.publish(p)
