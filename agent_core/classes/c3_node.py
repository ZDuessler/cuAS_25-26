import zmq
import threading
from abc import ABC, abstractmethod
from typing import Union, List, Optional
from datetime import datetime
import json
import os
import sys
import yaml
import time
import re
import subprocess
import platform

from classes.c3_node_message import C3NodeMessage
from classes.trigger import TriggerManager
# from classes.c3_node_utils import C3NodeUtils  # noqa: F401


'''
Contains the following Classes:

    - C3NodeMessage: This gives a standard format to the messages passed
        accross the node network
    - C3Node_v3: This is the main class for standalone C3Nodes.  These can
        exist onboard an airborne agent or on a ground station
    - DynamicMessageClass: This is a dynamic class that is used to define an
        agent's C3 node connection. Since each agent can have n-number of
        connections, this allows them to be created dynamically based on both
        YAML config layout and real-time C3 node updates inflight
    - AgentC3Node: This is a modified class of C3Node for the agents to
        implement. The concept of operations is that the AgentC3Node talks to a
        C3Node object if it wants to connect to the outside world.  The reason
        we disagregate this is to not overburden the agent's main logic loop
'''


def read_yaml_file(filename):
    '''
    This support funtion reads in the YAML file associated with the C3Node

    Args:
        `filename (str)`: The name of the YAML file.  Should be in the same
         folder as the [C3Nde].py file

    Return:
        dictionary of the YAML file
    '''
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Get the parent directory
    parent_dir = os.path.dirname(script_dir)

    # Construct the file path
    file_path = os.path.join(parent_dir, filename)
    with open(file_path, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            print("Unable to read/load the C3Node's YAML file")
            sys.exit()

    return data


class C3Node():
    '''
    The core class of a C3Node.  This can be used at a ground station or
    onboard a drone.  C3Nodes can talk to AgentC3Nodes as well as other
    C3Nodes.  It can take DIRECT messages through its ROUTER and it can
    BROADCAST messages through is PUBLISHER.  You can also simultaneously
    have it act like an Agent Node by giving it a C3_NODES dictionary.
    Through the agent nodes, it can receive and send DIRECT and BROADCAST
    messages.

    Attrs:
        `c3_id (str)`: The name of the C3Node.  It should match what is in the
            C3_NODES dictionary in the agents' YAML file
        `direct_port (int)`: The port on this device that agents' and C3Nodes'
            DIRECT connection will need to be connected to.
        `publish_port (int)`: The port on this device that agents' and C3Nodes'
            BROADCAST connection will need to be connected to.

    Vars:
        `identity (str)`: The name of this node as defined by the C3_ID
            in the yaml file
        `connected_clients ([str])`: List to keep track of connected clients
        `agent_c3_node_manager ([str])`: This C3Node’s AgentC3NodeManager
            for when it acts like an agent on the network vs. a C3Node


    Methods:
        - `send_direct_message(): Send a message to a specific agent (str)
            or group of agents [str, str, str, ...]
        - `send_broadcast_message(): Send a message to a specific subscriber
            group (str) or a group of subscriber groups [str, str, str, ...]
    '''

    def __init__(self,
                 config_file: str = None,
                 c3_id: str = None,
                 direct_port: int = None,
                 publish_port: int = None):

        # Make sure you have the minimum parameters to start the C3Node
        if config_file is None:
            if c3_id is None or direct_port is None or publish_port is None:
                raise ValueError("If config_file is not provided, " +
                                 "c3_id, direct_port, and publish_port " +
                                 "must be set.")
            else:
                self.identity = c3_id
                self._direct_port = direct_port
                self._publish_port = publish_port
        else:
            self._config_file = config_file
            self._config = read_yaml_file(config_file)
            # If this node is a C3 node, the ID is defined by C3_ID
            if "C3_ID" in self._config:
                self.identity = str(self._config['C3_ID'])
            # IF this node is an Agent node, the ID is defined by AGENT_ID
            elif "AGENT_ID" in self._config:
                self.identity = str(self._config['AGENT_ID'])

            # If this node has a direct port for receiving messages
            if "DIRECT" in self._config:
                self._direct_port = str(self._config['DIRECT'])
            # If this node has a publishing port for broadcasting messages
            if "PUBLISH" in self._config:
                self._publish_port = str(self._config['PUBLISH'])
            if "SEND_BUFFER_SIZE" not in self._config:
                self._config["SEND_BUFFER_SIZE"] = 10
            if "RECEIVE_BUFFER_SIZE" not in self._config:
                self._config["RECEIVE_BUFFER_SIZE"] = 10
            if "MAX_MSG_AGE" not in self._config:
                self._config["MAX_MSG_AGE"] = 2

        self._context = zmq.Context()
        self._poller = zmq.Poller()
        self._socket_dict = {}

        if "DIRECT" in self._config or direct_port is not None:
            # Create a ROUTER socket for bidirectional communication
            self._rtr_socket = self._context.socket(zmq.ROUTER)
            self._rtr_socket.setsockopt_string(zmq.IDENTITY, self.identity)
            self._rtr_socket.setsockopt(zmq.SNDHWM,
                                        self._config["SEND_BUFFER_SIZE"])
            self._rtr_socket.setsockopt(zmq.RCVHWM,
                                        self._config["RECEIVE_BUFFER_SIZE"])
            self._rtr_socket.bind(f"tcp://*:{self._direct_port}")
            self._socket_dict["ROUTER"] = self._rtr_socket
            self._poller.register(self._rtr_socket, zmq.POLLIN)

        if "PUBLISH" in self._config or publish_port is not None:
            # Create a PUB socket for publishing messages
            self._pub_socket = self._context.socket(zmq.PUB)
            self._pub_socket.setsockopt_string(zmq.IDENTITY, self.identity)
            self._pub_socket.bind(f"tcp://*:{self._publish_port}")
            self._socket_dict["PUBLISH"] = self._pub_socket
            self._poller.register(self._pub_socket, zmq.POLLIN)

        self._tab_pressed = False

        # List to keep track of connected clients
        self.connected_clients = {}

        # Dict of group variables
        self.c3node_var_dict = {}
        self.triggerMgr = TriggerManager(self)

        # Lock for thread-safe operations on connected_clients
        self.lock = threading.Lock()

        self.agent_c3_node_manager = AgentC3NodeManager(self._config)

        # self.establish_logic_objects()

    def start(self):

        # Start a background thread to handle incoming ROUTER messages
        self._run_thread = threading.Thread(target=self._run, daemon=False)
        self._run_thread.start()

        # if "PUBLISH"
        # Start a background thread to handle broadcast messages
        self._broadcast_thread = threading.Thread(
            target=self._c3_broadcast_function, daemon=True)
        self._broadcast_thread.start()

        # Start a background thread to handle terminal messages
        if os.name == 'nt':
            self._terminal_thread = threading.Thread(
                target=self._terminal_input_nt, daemon=True)
            self._terminal_thread.start()
        elif os.name == 'posix':
            self._terminal_thread = threading.Thread(
                target=self._terminal_input_posix, daemon=True)
            self._terminal_thread.start()

    def stop(self):
        # Unregister the socket and close it when stopping
        self._poller.unregister(self._rtr_socket)
        self._rtr_socket.close()

        self._poller.unregister(self._pub_socket)
        self._pub_socket.close()

    def _main_loop_thread(self):
        # while True:
        #     self.c3node_main_loop()
        self.c3node_main_loop()
        time.sleep(0.01)

    def c3node_main_loop(self):
        pass

    def _run(self):

        self.establish_logic_objects()

        # Continuous loop to handle incoming messages
        while True:

            # Run the main logic loop
            self._main_loop_thread()

            timeout = 50  # in msec
            # Poll for events (e.g., incoming messages) on registered sockets
            sockets = dict(self._poller.poll(timeout))

            if "ROUTER" in self._socket_dict:
                # Check if there is an incoming message on the
                # router socket poller
                if (self._rtr_socket in sockets and
                        sockets[self._rtr_socket] == zmq.POLLIN):
                    # Receive the multipart message:
                    #   3-part (intended for C3Node as there
                    #           is not a 'to' field):
                    #   [from_id, '', message]
                    #   7 part (intended for C3Node or other agents/groups):
                    #   [from_id, '', message, '',
                    #    message_type '', to_id/group_id]
                    #
                    #    - message_type can be DIRECT where to_id can be
                    #      a single client id or a list
                    #    - message_type can be BROADCAST where the group_id
                    #      can be single group or list

                    multipart_message = self._rtr_socket.recv_multipart()
                    if (not (len(multipart_message) != 3) or
                            (len(multipart_message) != 7)):
                        continue
                    received_from = multipart_message[0].decode('utf-8')
                    received_message = multipart_message[2].decode('utf-8')

                    # print(f"received_from: {received_from}")

                    try:
                        # Attempt to decode the byte string as JSON
                        message = json.loads(received_message)
                    except json.JSONDecodeError:
                        message = received_message

                    if len(multipart_message) == 3:
                        # This is a message from the client direct
                        # to the C3Node (no specified to_id)
                        c3Message = C3NodeMessage(
                            node_name=self.identity,
                            message=message,
                            sender=received_from
                        )

                        self._process_message(c3Message)

                    elif len(multipart_message) == 7:

                        # print(f"multipart_message: {multipart_message}")

                        # This is a passthrough message from the
                        # client to other clients either via DIRECT
                        # messaging or by BROADCAST (has a to_id)
                        message_type = multipart_message[4].decode('utf-8')
                        to_ids = multipart_message[6].decode('utf-8')
                        try:
                            # Attempt to decode the byte string as JSON
                            ids = json.loads(to_ids)
                        except json.JSONDecodeError:
                            # if it's not a JSON dict, then just
                            # make it plain string
                            ids = to_ids

                        # print(f"to_ids: {to_ids}")
                        # print(f"node_name: {self.identity}, "
                        #       f"message: {message}, "
                        #       f"message_type: {message_type}, "
                        #       f"sender: {received_from}, "
                        #       f"message_group: {ids}")
                        c3Message = C3NodeMessage(
                            node_name=self.identity,
                            message=message,
                            message_type=message_type,
                            sender=received_from,
                            message_group=ids)
                        # print(f"c3Message.to_dict(): {c3Message.to_dict()}")
                        self._process_message(c3Message, to_ids)

            # If the C3Node also acts as an agent on the network then
            # look at each Agentc3Node and see if there is a new message
            # disregarding the message if this agent sent it
            for Agentc3Node in self.agent_c3_node_manager.c3Nodes:
                if (Agentc3Node.is_new
                        and str(Agentc3Node.message.sender) !=
                        str(self._config['C3_ID'])):
                    # if the message is older than MAX_MSG_AGE seconds
                    # then don't accept it
                    try:
                        if (self.time_diff(message["timestamp"]) >
                                self._config["MAX_MSG_AGE"]):
                            return
                    except (KeyError, UnboundLocalError, TypeError):
                        pass

                    self.process_message_as_agent(Agentc3Node.message)

            time.sleep(0.01)  # allows the thread loop to complete

    def _update_group_var(self, c3Message):

        # print(f"msg: {c3Message.message}", flush=True)

        try:
            for c3node_var_key in self.triggerMgr.vars.keys():
                c3node_var: TriggerManager.C3NodeMessageVar =\
                    self.triggerMgr.vars[c3node_var_key]
                # print(f"C3NodeMessage: "
                #       f"{self.triggerMgr.vars[c3node_var_key].msg_dict}")
                if (isinstance(c3node_var, TriggerManager.C3NodeMessageVar) and
                        True):
                    # print(f"C3Message: {c3Message.to_dict()}")
                    c3node_var.update_c3node_var_by_c3m(
                        c3node_var_key, c3Message)

        except AttributeError:
            pass

    # ##################################################################################
    def _process_message(self,
                         c3Message: C3NodeMessage,
                         to_ids: Union[str, List[str]] = None):
        # Used to log new clients and then process the incoming messages
        # This method should be overwritten in the inherited class
        # Whereby the user should program how they want the data processed
        # if message_type is None:

        self._connect_client(c3Message.sender)

        # If the message has no to_id or the to_id is this C3Node
        if to_ids == self.identity or to_ids is None:
            # print(f"The message = {c3Message.to_dict()}")
            # #############################################################
            # #############################################################
            # ############# Update the following function #################
            self._update_group_var(c3Message)
            self.process_message_as_c3(c3Message)
        # else, send this message along
        else:
            if c3Message.message_type == "DIRECT":
                self.send_direct_message(
                    to_ids, c3Message.sender, c3Message.message)
            elif c3Message.message_type == "BROADCAST":
                self.send_broadcast_message(
                    to_ids, c3Message.sender, c3Message.message)
            elif c3Message.message_type == "C3_COMMAND":
                self.send_node_config_command(c3Message.message)

    @abstractmethod
    def establish_logic_objects(self):
        pass

    @abstractmethod
    def process_message_as_c3(self, message: C3NodeMessage):
        pass

    @abstractmethod
    def process_message_as_agent(self, c3_message: C3NodeMessage):
        pass

    def message_equals_value(
            self,
            message,
            value: Union[str, float, int],
            key: Optional[str] = None,
            msg_class: Optional[str] = None
            ):
        '''
        This function checks whether or not the message is equal to value

        Attr:
            `message (str, float, int, dict)`:
                The message can take on a multitude of types
            `value (str, float, int)`: the value to be matched to the message
            `key Optional (str)`: if the message is a dict, what key's value
                do you want to compare to this method's input 'value'
            `msg_class Optional (str)`: if you are looking at a dict of an
                agent_status parameter (agent_position, flight_mode, etc.)
                this is the name (str) of the expected message_class

        Return:
            True or False depending on whether the value matches the message
        '''

        if isinstance(message, (str, float, int)):
            if message == value:
                return True
        elif (isinstance(message, dict) and
                msg_class is not None):
            if "message_class" in message.keys():
                if message["message_class"] == msg_class:
                    if message[key] == value:
                        return True
        return False

    def message_is_of_class(self,
                            message,
                            msg_class):

        if isinstance(message, dict):
            if "message_class" in message.keys():
                if message["message_class"] == msg_class:
                    return True
            else:
                return False
        else:
            return False

    def message_string_equals(self, message, string):

        if not isinstance(message, str):
            return False
        elif message == string:
            return True
        else:
            return False

    def message_has_key(self, message, key):

        if not isinstance(message, dict):
            return False
        else:
            if key in message.keys():
                return True
            return False

    # ###################### Terminal Input ########################
    def _terminal_input_nt(self):

        import msvcrt
        while True:
            bkey = msvcrt.getch()
            key = bkey.decode('utf-8')
            if key == '\t':
                self._tab_pressed = True
                user_input = input("CMD: ")
                if user_input.lower() == 'exit':
                    os._exit(0)
                print(f"You entered: {user_input}")
                self._tab_pressed = False
                self.process_terminal_input(user_input)
            if key == 'q':
                os._exit(0)

    def _terminal_input_posix(self):
        '''
        Used for reading terminal inputs from within a Linux/MacOS machine
        '''

        import sys
        import select
        import tty
        import termios

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                if (select.select([sys.stdin], [], [], 0) ==
                        ([sys.stdin], [], [])):
                    key = sys.stdin.read(1)
                    if key == '\t':
                        self._tab_pressed = True
                        # Switch to cooked mode
                        termios.tcsetattr(
                            sys.stdin, termios.TCSADRAIN, old_settings)
                        user_input = input("CMD: ")
                        # Switch back to cbreak mode
                        tty.setcbreak(sys.stdin.fileno())
                        if user_input.lower() == 'exit':
                            # Switch to cooked mode
                            termios.tcsetattr(
                                sys.stdin, termios.TCSADRAIN, old_settings)
                            os._exit(0)
                        print(f"You entered: {user_input}")
                        self._tab_pressed = False
                        self.process_terminal_input(user_input)
                    if key == 'q':
                        # Switch to cooked mode
                        termios.tcsetattr(
                            sys.stdin, termios.TCSADRAIN, old_settings)
                        os._exit(0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    @abstractmethod
    def process_terminal_input(self, terminal_input: str):

        '''
        Override this command in inherited object:

        Manage custom terminal inputs here then return

        OR

        Allow send_config_command (below)
        to manage send messages to individual agents or
        groups based on the DIRECT_GROUPS and BROADCAST_GROUPS
        in the C3Node YAML file
        '''

        self.send_node_config_command(terminal_input)

    def send_node_config_command(self, command_message: str):
        '''
        Send the command as defined in the c3Node.config YAML 'COMMAND' section

        Format for DIRECT: 'cd[agent_ids] config_command'
            - `cd1 launch`
            - `cd[80001, 80002] [cmd_sys_mode_change(guided),
                                cmd_sys_arm_disarm(arm, True 15),
                                cmd_nav_guided_takeoff(10)]`

        Format for BROADCAST: 'cb[group_ids] config_command'
            - `cb1 launch`
            - `cd[group1, ALLSUBSCRIBERS] [cmd_sys_mode_change(guided),
                                          cmd_sys_arm_disarm(arm, True 15),
                                          cmd_nav_guided_takeoff(10)]`
        '''

        commands_dict = read_yaml_file(self._config_file)['COMMANDS']

        def condition_cmd_string(input_string):
            # Replace spaces within square and round brackets
            input_string = re.sub(
                r'\[\s*([^]]*)\s*\]',
                lambda m: '[' + m.group(1).replace(' ', '') + ']',
                input_string)
            # Replace spaces within parentheses
            input_string = re.sub(
                r'\(\s*([^)]*)\s*\)',
                lambda m: '(' + m.group(1).replace(' ', '') + ')',
                input_string)
            return input_string

        def resolve_values(d, initial_key):
            import re

            def parse_key(key):
                match = re.match(r"(\w+)\((.*?)\)", key)
                if match:
                    name = match.group(1)
                    params = match.group(2).split(',')
                    params = [s.replace(" ", "") for s in params]
                    return name, params
                return key, []

            def substitute_params(pattern, params, values):
                for i, param in enumerate(params):
                    pattern = pattern.replace(param, values[i])
                return pattern

            def resolve_key(key, visited):
                name, param_values = parse_key(key)
                resolved_keys = []

                # Handle parameterless keys
                if not param_values:
                    if key in visited:
                        raise ValueError(f"Cycle detected: {key} has already "
                                         f"been visited.")
                    visited.add(key)

                    if key in d:
                        values = d[key]
                        if isinstance(values, str):
                            values = [values]

                        for val in values:
                            resolved_keys.extend(resolve_key(val, visited))
                    else:
                        resolved_keys.append(key)

                    visited.remove(key)
                    return resolved_keys

                # Handle parameterized keys
                for dict_key in d.keys():
                    dict_name, dict_params = parse_key(dict_key)
                    if name == dict_name and \
                            len(param_values) == len(dict_params):
                        if key in visited:
                            raise ValueError(
                                f"Cycle detected: {dict_key} has already "
                                f"been visited.")
                        visited.add(dict_key)

                        values = d[dict_key]
                        if isinstance(values, str):
                            values = [values]

                        for val in values:
                            new_key = substitute_params(
                                val, dict_params, param_values)
                            resolved_keys.extend(resolve_key(new_key, visited))

                        visited.remove(dict_key)
                        return resolved_keys

                return [key]

            return resolve_key(initial_key, set())

        def remove_first_word_in_values(my_dict):
            for key in my_dict:
                if isinstance(my_dict[key], str)\
                        and (my_dict[key].startswith('cd')
                             or my_dict[key].startswith('cb')):
                    # print(f"The value to delete: {my_dict[key]}")
                    my_dict[key] = ''.join(my_dict[key].split(' ')[1:])
                elif isinstance(my_dict[key], list):
                    new_list = []
                    for item in my_dict[key]:
                        if isinstance(item, str)\
                                and (item.startswith('cd')
                                     or item.startswith('cb')):
                            # print(f"The value to delete: {item}")
                            new_list.append(''.join(item.split(' ')[1:]))
                        else:
                            new_list.append(item)
                    my_dict[key] = new_list
            return my_dict

        # Condition the command by putting removing spaces within elements
        # It retains spaces between the addressees and each command
        command_message = condition_cmd_string(command_message)
        # print(f"The command_message is: {command_message}")

        receivers_list = []
        cmd_line = command_message
        cmd_type = None

        # If you are directly assigning receivers
        if command_message[:2] == "cd" or command_message[:2] == "cb":

            commands_dict = remove_first_word_in_values(commands_dict)

            # Get receiver(s)
            cmd_line_list = command_message.split(" ")
            receivers = cmd_line_list[0][2:]

            # if you have a list of receivers
            if receivers[0] == "[":
                receivers = receivers[1:-1]
                try:
                    receivers_list = receivers.split(",")
                except (ValueError, SyntaxError) as e:
                    print(f"An error occurred while parsing the list: {e}")
                    return

            # else single receiver
            # Put the single receiver in a list (receiver_list)
            else:
                if command_message[:2] == "cd":
                    try:
                        receivers_list = \
                            self._config["DIRECT_GROUPS"][receivers]
                    except KeyError as e:
                        print(f"{e} is not a name of a custom DIRECT_GROUP")
                        return
                if command_message[:2] == "cb":
                    try:
                        receivers_list = \
                            self._config["BROADCAST_GROUPS"][receivers]
                    except KeyError as e:
                        print(f"{e} is not a name of a custom BROADCAST_GROUP")
                        return

            cmd_line_list = cmd_line_list[1:]

            if command_message[:2] == "cd":
                cmd_type = 'DIRECT'
            elif command_message[:2] == "cb":
                cmd_type = 'BROADCAST'

        else:
            cmd_line_list = cmd_line.split(' ')

        full_command_list = []
        for cmd in cmd_line_list:
            sub_cmd_list = resolve_values(commands_dict, cmd)
            full_command_list.extend(sub_cmd_list)

        if cmd_type == 'BROADCAST' or cmd_type == 'DIRECT':

            print(f"receivers: {receivers_list} - "
                  f"command_str: {full_command_list}")

            command_str = json.dumps(full_command_list)
            # ### Send the receivers a json message with all the commands
            if cmd_type == 'DIRECT':
                self.send_direct_message(
                    receivers_list, self.identity, command_str)
            elif cmd_type == 'BROADCAST':
                self.send_broadcast_message(
                    receivers_list, self.identity, command_str)
            return
        else:
            # ### Itereate through the local commands sending each
            # ### one one at a time
            for cmd in full_command_list:
                # if cmd not in commands_dict.keys():
                #     print(f"5 - '{cmd}' not found in "
                #           f"{self.identity}'s [COMMANDS]")
                #     return
                self.send_node_config_command(cmd)

    # ####################### Broadcast Mgt ########################
    def _c3_broadcast_function(self):
        while True:
            self.c3_broadcast_loop()
            time.sleep(0.01)  # allows the thread loop to complete

    @abstractmethod
    def c3_broadcast_loop(self):
        pass

    # #####################################################################################
    def send_direct_message(self,
                            to_id: Union[str, List[str]],
                            from_id: str,
                            message: Union[str, List[str], dict]):
        '''
        Send a message to the Agent/C3 ID that is in to_id
        '''
        # Send a direct message to the id or [ids] through the ROUTER

        # print(f"Send to: {to_id} in {self.connected_clients}")

        if isinstance(message, str):
            msg_list = [message]
        elif isinstance(message, dict):
            msg_list = [json.dumps(message)]
        elif isinstance(message, list):
            msg_list = list(message)

        if not isinstance(to_id, list):
            to_id = [to_id]

        for id in to_id:
            # if isinstance(message, dict):
            #     message = json.dumps(message)
            for msg in msg_list:

                # print(f"to_id: {to_id} / msg: {msg}")

                self._rtr_socket.send_multipart([
                    str(id).encode('utf-8'),
                    b'',
                    str(from_id).encode('utf-8'),
                    b'',
                    msg.encode('utf-8')
                ])
        # else:
        #     if isinstance(message, dict):
        #         message = json.dumps(message)
        #     self._rtr_socket.send_multipart([
        #         str(to_id).encode('utf-8'),
        #         b'',
        #         str(from_id).encode('utf-8'),
        #         b'',
        #         message.encode('utf-8')
        #     ])

    def send_broadcast_message(self,
                               to_id: Union[str, List[str]],
                               from_id: str,
                               message: Union[str, dict]):
        # print("Here")
        # print(self._socket_dict)
        # If the publisher is activated
        if "PUBLISH" in self._socket_dict:
            # print("PUBLISH here")
            # Send a broadcast message to the id or [ids] through the PUBLISHER
            if isinstance(to_id, list):
                for id in to_id:
                    self._pub_socket.send_multipart([
                        str(id).encode('utf-8'),
                        b'',
                        str(from_id).encode('utf-8'),
                        b'',
                        message.encode('utf-8')
                    ])
            else:
                self._pub_socket.send_multipart([
                        str(to_id).encode('utf-8'),
                        b'',
                        str(from_id).encode('utf-8'),
                        b'',
                        message.encode('utf-8')
                    ])

    def _connect_client(self, client_identity: str):
        # Add a client identity to the list of connected clients
        with self.lock:
            self.connected_clients[client_identity] = \
                datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")

    def _disconnect_client(self, client_identity: str):
        # Remove a client identity from the list of connected clients
        with self.lock:
            if client_identity in self.connected_clients.keys():
                del self.connected_clients[client_identity]

    # Manage printing to the screen
    def tprint(self, input):
        if os.name == 'nt':
            if not self._tab_pressed:
                print(input)
            elif input == '':
                return

    def time_diff(self, time_str: str):
        current_time = datetime.now()
        time_to_check = datetime.strptime(time_str, "%m/%d/%Y, %H:%M:%S.%f")
        delta = (current_time - time_to_check).seconds
        return delta

    def kill_process_by_name(self, name):
        if str(platform.system()) == 'Windows':
            subprocess.run(
                'taskkill /F /IM mavproxy*',
                shell=True
            )
        elif str(platform.system()) == 'Linux':
            subprocess.run(
                'pkill -f mavproxy.py',
                shell=True
            )


class AgentC3NodeManager():

    def __init__(self, config: dict):

        self._config = config
        self.c3Nodes = []
        self._initialize_agent_hub_internal_zmq_connections()

    def _initialize_agent_hub_internal_zmq_connections(self):
        # ######## Setup ZMQ Connections to internal processes #########
        # Setup the zmq context and initialize the zmq parameters

        for key, value in self._config.get('C3_NODES', {}).items():
            dynamic_class = type(
                    key,
                    (AgentC3Node,),
                    {}
                )
            setattr(self, key, dynamic_class
                    (self._config, c3_node_name=key))

            # Instantiate an object of the dynamically created class
            dynamic_instance = getattr(self, key)
            self.c3Nodes.append((dynamic_instance))
            dynamic_instance.start()


class DynamicNodeClass(ABC):

    def __init__(self, **kwargs):
        self._is_new = True  # Flag to track if the data is new
        # self.timestamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        # self.update_attributes(kwargs)

    def to_dict(self):
        excluded_vars = {
                '_is_new',
                '_agent_hub',
                '_config',
                '_message_type',
                'identity',
                'context',
                'socket',
                'sub_socket',
                'poller'
                # 'timestamp'
            }
        return {
            key: value for key, value in vars(self).items()
            if key not in excluded_vars}

    @property
    def is_new(self):
        result = self._is_new
        self._is_new = False  # Auto reset when accessed
        return result


class AgentC3Node(DynamicNodeClass):
    '''
    This represents the Agent's C3 node(s).  They can receive messages
    from a centralized C3 node running on the same device or
    off-board.  They are auto-generated from the C3_NODES dictionary
    in the YAML file.  The AgentC3Node.name will interact with the
    C3Node of the same name.  It is important that the DIRECT and
    SUBSCRIBE sockets match those defined in the individual C3Node object

    Args:
        `config (dict)`: The agent's config dictionary
        `c3_node_name (str):` The name of the C3 node as defined in
         the YAML file's C3_NODES dictionary

    Methods:
        -`send_direcet_message()`: Send a message (str or dict) to specific
            AGENT_IDs or C3_IDS (str or [str])
        -`send_broadcast_message()`: Send a message (str or dict) to specific
            group (str or [str])
        -`send_command_message(str)`: Send a message to the C3 node.

    Return: None
    '''

    def __init__(self,
                 config: dict,
                 c3_node_name: str = None
                 ):

        super().__init__()  # Call the base class's __init__ method

        self._config = config
        self.node_name = c3_node_name
        _c3_dict = self._config["C3_NODES"]

        if "AGENT_ID" in self._config:
            self.identity = str(self._config["AGENT_ID"])
        elif "C3_ID" in self._config:
            self.identity = str(self._config["C3_ID"])
        self.context = zmq.Context()
        self.poller = zmq.Poller()
        self._socket_dict = {}

        # If the port exists, then create the socket connections
        if "DIRECT" in _c3_dict[c3_node_name]:
            # Create a DEALER socket for bidirectional communication
            _dlr_socket_address = _c3_dict[c3_node_name]["DIRECT"]["SOCKET"]
            self._dlr_socket = self.context.socket(zmq.DEALER)
            self._dlr_socket.setsockopt_string(zmq.IDENTITY, self.identity)
            # self._dlr_socket.setsockopt(zmq.SNDHWM, 10)
            self._dlr_socket.connect(_dlr_socket_address)
            self._socket_dict["DEALER"] = self._dlr_socket
            self.poller.register(self._dlr_socket, zmq.POLLIN)

        if "SUBSCRIBE" in _c3_dict[c3_node_name]:
            # Create a SUB socket for subscribing to published messages
            _sub_socket_address = _c3_dict[c3_node_name]["SUBSCRIBE"]["SOCKET"]
            list_of_subscriptions = \
                _c3_dict[c3_node_name]["SUBSCRIBE"]["GROUP_ID"]
            self._sub_socket = self.context.socket(zmq.SUB)
            for subscription in list_of_subscriptions:
                self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, subscription)
            self._sub_socket.connect(_sub_socket_address)
            self._socket_dict["SUBSCRIBE"] = self._sub_socket
            self.poller.register(self._sub_socket, zmq.POLLIN)

        self._is_new = False
        self.message = None

    def start(self):
        # Start the run loop in a separate thread
        run_thread = threading.Thread(target=self.run, daemon=True)
        run_thread.start()

    def stop(self):
        # Close the sockets when stopping the script
        for socket in self._socket_list:
            socket.close()

    def send_direct_message(self,
                            message: Union[str, dict, ABC],
                            to_id: Union[str, List[str]] = None):
        '''
        Send a multipart message to the CentralizedManager
        that should be sent directly to the id in to_id.
        The code can also pass a list of ids in to_id
        that will be parsed at the CentralizedManager and
        sent messages individually
        '''

        # If you don't have an assigned to_id, then assign
        # the message to the parent C3 Node
        if to_id is None:
            to_id = self.node_name

            if isinstance(message, ABC):
                message = message.to_dict()
        self._send_message(message, "DIRECT", to_id)

    def send_broadcast_message(self,
                               message: Union[str, dict],
                               group_id: Union[str, List[str]] = None):
        '''
        Send a multipart message to the CentralizedManager
        that should be broadcast to the subscribers on the
        subscribers list
        '''

        # If you don't have an assigned group, then assign
        # the message to the parent C3 Node
        if group_id is None:
            group_id = self.node_name
        self._send_message(message, "BROADCAST", group_id)

    def send_command_message(self,
                             command: str):
        '''
        Send a command message to the C3_Node.  This will run commands out
        of the C3_Node's 'COMMANDS' key in the C3_Node's config dictionary

        Attributes:
            `command (str)`: The command should match a command in the
                dict
        '''
        self._send_message(command, "C3_COMMAND")

    def _send_message(self,
                      message: Union[str, dict],
                      message_type: str = None,
                      ids: Union[str, List[str]] = None):
        '''
        Send a message to the C3 node this AgentC3Node is attached to
        '''

        def is_utf8_encoded(data):
            try:
                data.decode('utf-8')
                return True
            except UnicodeDecodeError:
                return False
            except AttributeError:
                return False

        if isinstance(message, ABC):
            message = message.to_dict()

        if ids is not None:
            if isinstance(ids, list):
                ids = json.dumps(ids)
            elif isinstance(ids, int):
                ids = str(ids)
        else:
            ids = ''

        if not isinstance(message, str):
            message = json.dumps(message)

        if message_type is not None:

            self._dlr_socket.send_multipart([
                b'',
                message.encode('utf-8'),
                b'',
                message_type.encode('utf-8'),
                b'',
                ids.encode('utf-8')
            ])
        else:
            if not is_utf8_encoded(message):
                message = message.encode('utf-8')
            # print(f"Message: {message}")
            self._dlr_socket.send_multipart([b'', message])

    def run(self):
        # Continuous loop to handle incoming messages
        while True:
            timeout = 100  # in msec
            sockets = dict(self.poller.poll(timeout))

            if "DEALER" in self._socket_dict:
                # Check if there is an incoming message on the DEALER socket
                if (self._dlr_socket in sockets and
                        sockets[self._dlr_socket] == zmq.POLLIN):
                    # Receive the multipart message: [to_id, from_id, message]
                    message = self._dlr_socket.recv_multipart()
                    self.message = C3NodeMessage(
                        node_name=self.node_name,
                        message=message,
                        message_type="DIRECT"
                    )
                    self._is_new = True

            if "SUBSCRIBE" in self._socket_dict:
                # Check if there is an incoming published message
                # on the SUB socket
                if (self._sub_socket in sockets and
                        sockets[self._sub_socket] == zmq.POLLIN):
                    # Receive a published message from the PUB socket:
                    # [to_id, from_id, message]
                    message = self._sub_socket.recv_multipart()
                    self.message = C3NodeMessage(
                        node_name=self.node_name,
                        message=message,
                        message_type="BROADCAST"
                    )
                    self._is_new = True
