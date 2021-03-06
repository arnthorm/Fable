
from parser import MessageParser

class MessageManager(object):
  """
  Manager to handle message flow between embedded device and a computer.

  A event driven manager that handles message subscription and publishing.
  A given transport handles the raw communication to the device.
  """
  def __init__(self, transport):
    self.subscriptions = {}
    self.message_parser = MessageParser()
    self.transport = transport

  def reset(self):
    """Reset all subscriptions and clear all messages."""
    self.subscriptions = {}
    self.message_parser.reset()

  def act(self):
    """
    Act to received messages.

    A function that is supposed to run in endless loop
    to chech if a message has been received via
    transport and publishes to subscripers.
    """
    response = self.transport.receive()
    if response is not None:
      self._process_response(response)

  def send(self, msg_type, **kwargs):
    """
    Sends a message via transport.

    Converts a dictionary formed message to pre-defined
    struct like message and sends via given transport.

    Keyword arguments:
      msg_type -- Message type id
      kwargs -- Dictionary containing fields and values
                for the message.
    """
    data = self.message_parser.make(msg_type, kwargs)
    self.transport.send(data)

  def register_message(self, msg_type, description):
    """
    Register message type.

    Keyword arguments:
        msg_type    -- Id of message
        description -- Description in dictionary format: 
        {
          'name': 'Packet', 
          'fields': ('field1', 'field2',...), 
          'types': ('b', 'f', ...))
        }
    """
    self.message_parser.add(msg_type, description)

  def register_messages(self, collection):
    """Register a collection of message descriptions."""
    self.message_parser.add_collection(collection)

  def get_message_description(self, msg_type):
    """Gets a message description."""
    return self.message_parser.get_message_description(msg_type)

  def subscribe(self, msg_type, function=None):
    """Subscribe to a message type."""
    if not self.subscriptions.has_key(msg_type):
      self.subscriptions[msg_type] = [function]
    else:
      self.subscriptions[msg_type].append(function)

  def _process_response(self, response):
    """Processes a response and publishes a message to subscribers."""
    message = self.message_parser.parse(response)
    if message is not None:
      msg_type = message.msg_type
      if msg_type in self.subscriptions:
        for listener in self.subscriptions[msg_type]:
          listener(message)
    else:
      msg_type = None
      if msg_type in self.subscriptions:
        for listener in self.subscriptions[msg_type]:
          listener(response)

  def stop(self):
    """Stop listening for messages."""
    self.transport.stop()

