#!/usr/bin/env python
import rospy
from embedded.messages import MessageManager, SerialTransport
from embedded.helpers import attrs_to_dict, attrs_to_attrs, get_attr, set_attr

MSG_SUBSCRIBE_TOPIC = 140
MSG_SUBSCRIBE_TOPIC_FIELD = 141
MSG_PUBLISH_TOPIC = 142
MSG_RESET = 143

msgs_collection = {
  MSG_SUBSCRIBE_TOPIC: {
    'name': 'subscribe_topic',
    'fields': ('type', 'field'),
    'types': ('B', 'B'),
  },
  MSG_SUBSCRIBE_TOPIC_FIELD: {
    'name': 'subscribe_topic_field',
    'fields': ('type', 'field'),
    'types': ('B', 'B'),
  },
  MSG_PUBLISH_TOPIC: {
    'name': 'publish_topic',
    'fields': ('type', 'field'),
    'types': ('B', 'B'),
  },
  MSG_RESET: {
    'name': 'reset',
    'fields': (),
    'types': ()
  }
}


class ROSEmbeddedManager(MessageManager):
  """
  Manager to handle connection between ROS and embedded device.

  With this manager one can define connection between embedded device 
  and Robot Operating System (ROS). This manager and an embedded device 
  communicate with simple struct like messages which are defined in a 
  dictionary:
    {'name: 'MessageName', 'fields': ('field1', 'field2), 'types': ('B', 'B')}
  where the types are the same as in the python struct module.
  With the function link() one can describe how a embedded message
  is linked to a ROS topic/message.
  """
  def __init__(self, controller_setup, *args, **kwargs):
    super(ROSEmbeddedManager, self).__init__(*args, **kwargs)
    self._controller_setup = controller_setup
    self.ros_subscribers = []
    self.reset()

  def reset(self):
    """
    Reset manager.
    
    Reset manager by removing all links and subscriptions and setup again.
    """
    for sub in self.ros_subscribers:
      sub.unregister()

    super(ROSEmbeddedManager, self).reset()

    self.field_subscription = {}
    self.field_table = {}
    self.field_conv_table = {}

    self.register_messages(msgs_collection)

    self.subscribe(MSG_SUBSCRIBE_TOPIC, self.subscribe_topic_cb)
    self.subscribe(MSG_SUBSCRIBE_TOPIC_FIELD, 
                           self.subscribe_topic_field_cb)
    self.subscribe(MSG_RESET, self.reset_cb)
    self.subscribe(MSG_PUBLISH_TOPIC, self.publish_topic_cb)
    self._controller_setup(self)

  def add_field_table(self, field_table):
    """
    Add field table to manager.
    
    A field table is a dictionary where a field (as value) gets 
    a unique id (as key).

    Keyword arguments:
      field_table -- A dictionary containing numerical id (key)
                     and a field name (value).
    """
    self.field_table.update(field_table)

  def add_field_conv_table(self, field_table):
    """
    Add a field conversion table

    A field conversion table is a dictionary of field id (key) (see field table)
    and a conversion function (value), where the conversion function 
    converts the ROS field to embedded field.
    """
    self.field_conv_table.update(field_table)

  def link(self, msg_type, ros_type, topic_subscribe=None, 
           topic_publish=None, field_map=None, ros_emb_conv=None):
    """
      Link embedded messages to ROS topic messages.

      Keyword arguments:
        msg_type        -- Embedded message type.
        ros_type        -- ROS type of the message.
        topic_subscribe -- ROS topic that embedded message should subscribe to.
        topic_publish   -- ROS topic that embedded message should publish to.
        field_map       -- Dictionary containing map from embedded message to 
                           ROS topic field
    """
    desc = self.get_message_description(msg_type)
    if desc is not None:
      desc['ROS_type'] = ros_type
      desc['ROS_topic_publish'] = topic_publish
      desc['ROS_topic_subscribe'] = topic_subscribe
      if field_map is not None:
        desc['ROS_field_map'] = field_map
      if ros_emb_conv is not None:
        desc['ROS_emb_conv'] = ros_emb_conv
    else:
      rospy.logerr("Message with id %d does not exists." % msg_type)

  def reset_cb(self, message):
    """Reset manager, a callback for reset message."""
    rospy.loginfo("Reseting ...")
    self.reset()

  def convert_ROS_to_embedded(self, desc, data):
    """
    Converts ROS topic/message to embedded message.
    
    Keyword arguments:
      desc -- Description of embedded message.
      data -- ROS message.
    """
    if desc.has_key('ROS_field_map'):
      # There is a map between ROS messsage and embedded message:
      d = attrs_to_dict(data, desc['fields'], desc['ROS_field_map'])
    else:
      d = attrs_to_dict(data, desc['fields'])
    if desc.has_key('ROS_emb_conv'):
      for key in desc['ROS_emb_conv'].keys():
        d[key] = desc['ROS_emb_conv'][key](d[key])
    return d
  
  def subscribe_topic_cb(self, message):
    """
    Subscribe to ROS topic, callback.

    Message from embedded device to subscribe to ROS topic,
    according to pre-defined link (see self.link(...)).
    """
    desc = self.get_message_description(message.type)
    if desc.has_key('ROS_type') and desc.has_key('ROS_topic_subscribe'):
      def callback(data):
        d = self.convert_ROS_to_embedded(desc, data)
        self.send(message.type, **d)
      self.ros_subscribers.append(rospy.Subscriber(
        desc['ROS_topic_subscribe'], 
        desc['ROS_type'], 
        callback,
      ))
      rospy.loginfo("Message \"%s\" (%d) is subscribed to ROS topic %s." 
        % (desc['name'], message.type, desc['ROS_topic_subscribe']))
    else:
      rospy.logerr("Message not supported \"%s\" for subscription." 
                   % desc['name'])

  def subscribe_topic_field_cb(self, message):
    """
    Subscribe to ROS topics specific field.

    Keyword arguments:
      message.type  -- Embedded message type.
      message.field -- ROS message field id (from field_table).
    """
    desc = self.get_message_description(message.type)
    if desc.has_key('ROS_type') and desc.has_key('ROS_topic_subscribe'):
      if not self.field_subscription.has_key(message.type):
        self.field_subscription[message.type] = []
        def callback(data):
          # Get field names ('/left_arm/sholder_pitch') from defined number (1)
          fields_num = self.field_subscription[message.type]
          fields_name = (
            self.field_table[field] 
            for field in fields_num
          )

          # Convert individual fields values
          for field_num in fields_num:
            if self.field_conv_table.has_key(field_num):
              value = get_attr(data, self.field_table[field_num])
              value = self.field_conv_table[field_num](value)
              set_attr(data, self.field_table[field_num], value)

          # Extract appropriate fields from the ROS message 
          if desc.has_key('ROS_field_map'):
            # There is a map between ROS messsage and embedded message.
            # Convert from ROS value to embedded value:
            d = attrs_to_dict(data, desc['fields'], desc['ROS_field_map'])
          else:
            d = attrs_to_dict(data, desc['fields'])

          d['array'] = []
          for field in fields_name:
            d[desc['fields'][-1]].append(get_attr(data, field))
          d['length'] = len(d[desc['fields'][-1]])
          if desc.has_key('ROS_emb_conv'):
            for key in desc['ROS_emb_conv'].keys():
              d[key] = desc['ROS_emb_conv'][key](d[key])
          self.send(message.type, **d)
        self.ros_subscribers.append(rospy.Subscriber(
          desc['ROS_topic_subscribe'],
          desc['ROS_type'], 
          callback
        ))
        rospy.loginfo("Message \"%s\" (%d) is subscribed to ROS topic %s." 
          % (desc['name'], message.type, desc['ROS_topic_subscribe']))
      self.field_subscription[message.type].append(message.field)
    else:
      rospy.logerr("Message not supported \"%s\" for subscription." 
                   % desc['name'])

  def publish_topic_cb(self, message):
    """
    Publish embedded message to ROS topic.

    According to pre-specified link (self.link(...)).

    Keyword arguments:
      message.type -- Embedded message type.
    """
    desc = self.get_message_description(message.type)
    if desc.has_key('ROS_type') and desc.has_key('ROS_topic_publish'):
      pub = rospy.Publisher(desc['ROS_topic_publish'], desc['ROS_type'])
      def callback(data):
        object = desc['ROS_type']()
        attrs_to_attrs(object, desc['fields'], data)
        pub.publish(object)
      self.subscribe(message.type, callback)
      rospy.loginfo("Message \"%s\" (%d) will be published to %s." \
        % (desc['name'], message.type, desc['ROS_topic_publish']))
    else:
      rospy.logerr("Message not supported: %s" % desc['name'])

