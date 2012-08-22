
from embedded.helpers import enum
from math import pi

msgs = enum(
  echo=150, 
  say=151,
  user_detected=152,
  user_lost=153,
  user_recognized=154,
  user_pos=155,
  user_ang=156,
  posture_detected=157,
  motor_subscription=158,
  motor_command=159
)

field_table = {
  0: '/left_arm/elbow_pitch/angle',
  1: '/left_arm/shoulder_pitch/angle',
  2: '/left_arm/shoulder_yaw/angle',
  3: '/left_arm/sholder_roll/angle',
  4: '/right_arm/elbow_pitch/angle',
  5: '/right_arm/shoulder_pitch/angle',
  6: '/right_arm/shoulder_yaw/angle',
  7: '/right_arm/sholder_roll/angle',
  8: '/left_leg/knee_pitch/angle',
  9: '/left_leg/hip_pitch/angle',
  10: '/left_leg/hip_yaw/angle',
  11: '/left_leg/hip_roll/angle',
  12: '/right_leg/knee_pitch/angle',
  13: '/right_leg/hip_pitch/angle',
  14: '/right_leg/hip_yaw/angle',
  15: '/right_leg/hip_roll/angle',
}

def field_ang_conv(from_value, to_value):
  range = to_value - from_value
  if to_value > from_value:
    upper = to_value
    lower = from_value
  else:
    upper = from_value
    lower = to_value

  def conv(value):
    if range < 0:
      value = pi - value
    value = (value-lower)/abs(range)*pi
    if value < 0:
      value = 0
    elif pi < value:
      value = pi
    return value
  return conv

field_conv_table = {
  0: field_ang_conv(0.4, 2.8), # left elbow 
  1: field_ang_conv(pi/2, pi), # left shoulder pitch
  2: field_ang_conv(0.2, 2.8), # left shoulder yaw
  4: field_ang_conv(0.4, 2.7), # right elbow
  5: field_ang_conv(pi/2, pi), # right shoulder pitch
  6: field_ang_conv(0.2, 2.8), # right shoulder yaw
}

"""
Description of messages

Each description of message has a counter definition on the embedded device
which is defined using a C struct object. The description is consists of 
name, field names and field types. The field types are defined in a similar
maner as the python's struct package.
"""
messages_collection = {
  msgs.echo: {
    'name': 'Echo',
    'fields': ('length', 'data'),
    'types': ('B', '%ds'),
    'has_array': True
  },
  msgs.user_detected: {
    'name': 'UserDetected',
    'fields': ('user_id',),
    'types': ('B',),
  },
  msgs.user_lost: {
    'name': 'UserLost',
    'fields': ('user_id',),
    'types': ('B',),
  },
  msgs.user_recognized: {
    'name': 'UserRecognized',
    'fields': ('user_id', 'length', 'username'),
    'types': ('B', 'B' '%ds'),
    'has_array': True
  },
  msgs.user_pos: {
    'name': 'UserPos',
    'fields': ('user_id', 'length', 'array'),
    'types': ('B', 'B', '%dB'),
    'has_array': True,
  },
  msgs.user_ang: {
    'name': 'UserAng',
    'fields': ('user_id', 'length', 'array'),
    'types': ('B', 'B', '%dH'),
    'has_array': True,
  },
  msgs.posture_detected: {
    'name': 'PostureDetected',
    'fields': ('user_id', 'posture'),
    'types': ('B', 'B'),
    'has_array': True
  },
  msgs.say: {
    'name': 'Say',
    'fields': ('length', 'array',),
    'types': ('B', '%ds',),
    'has_array': True
  },
  msgs.motor_subscription: {
    'name': 'MotorSubscription',
    'fields': ('motor_num',),
    'types': ('B',)
  },
  msgs.motor_command: {
    'name': 'MotorCommand',
    'fields': ('motor_id', 'pos'),
    'types': ('B', 'H')
  }
}
