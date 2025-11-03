from typing import Union
from datetime import datetime


class C3NodeMessage():
    '''
    Stores the latest AgentC3Node's C3 message in an object.

    Vars:
        `timestamp (str)`: The time the message was received
        `node_name (str)`: The name of the C3 node to which the
         message belongs.
        `sender (str)`: The id of the node/agent sending the message
        `message (str)`: The actual message from the node/agent
        `message_type (str)`: The type of message (DIRECT/BROADCAST)
        `message_group (str)`: If BROADCAST, which group_id accepted
         the message.

    Methods:
        - `to_dict()`: Converts the object to a dictionary
    '''

    def __init__(self,
                 node_name: str = None,
                 message: Union[list[str], str] = None,
                 message_type: str = None,
                 sender: str = None,
                 message_group: Union[list[str], str] = None):

        self.timestamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        self.node_name = node_name

        if isinstance(message, list):
            if message[0].decode('utf-8') == "":
                self.sender = message[1].decode('utf-8')
                self.message = message[3].decode('utf-8')
                self.message_type = "DIRECT"
                self.message_group = None
            else:
                self.sender = message[2].decode('utf-8')
                self.message = message[4].decode('utf-8')
                self.message_type = "BROADCAST"
                self.message_group = message[0].decode('utf-8')

        elif isinstance(message, str) or isinstance(message, dict):
            self.sender = sender
            self.message = message
            self.message_type = message_type
            if message_type == "DIRECT":
                self.message_group = None
            elif message_type == "BROADCAST":
                self.message_group = message_group

    def to_dict(self):
        excluded_vars = {
                # 'timestamp'
            }
        return {
            key: value for key, value in vars(self).items()
            if key not in excluded_vars}
