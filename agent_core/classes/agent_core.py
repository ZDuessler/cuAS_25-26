import sys
import time
import threading
import argparse
import ast
import re
import json

from classes.agentutils import AgentUtils
from classes.agent_hub import AgentHub
from agent_status_class import AgentStatus
from classes.c3_node_message import C3NodeMessage


class AgentCore:
    '''
    This is the main class for Agent Core that contains the agent's status
    object, runs the main logic loop and listens for C3 node inputs for
    this drone. To create an Agent, you want to inherit
    this class and override the following functions:
        - agent_core_loop_functions(self)
        - process_c3_input(self, c3_message: C3NodeMessage)

    Args:
        `loop_delay (float)`: Set a delay in the main loop function
        `configuration_file (str)`: The realtive path+name of the
            configuration YAML file
        `log_file (bool)`:
        `verbose (bool)`: Print to terminal the expanded data/debug lines

    Params:
        `terminal_input (str)`: The UI string input from the terminal
        `config (dict)`: This is a dict of the configuration YAML
        `agent_hub (AgentHub)`: Reference to the agent's AgentHub
        `agent_status (AgentStatus)`: Reference to the agent's status object
        `acm (AgentCommandManager)`: Reference to the agent's Command Manager
        `c3nm (AgentC3NodeManager)`: Reference to the agent's C3 Node Manager
        `stopFlag (threading.Event())`: A flag to stop the main loop.
            stopFlag.set() stops the loop and stopFlag.clear() starts the loop

    Methods (to override in your inherited AgentCore class):
        - `agent_core_loop_functions()`: This is the function that listens for
            and reports changes to the agent_status object
        - `process_c3_input(self, c3_message: C3NodeMessage)`: This is the
            function that processes messages received from any of the C3 nodes

    Return: None
    '''

    # class TimeKeeper:
    #     def __init__(self, interval, action):
    #         self.interval = interval
    #         self.action = action
    #         self.thread = threading.Thread(target=self.run)
    #         self.thread.daemon = True
    #         self.thread.start()

    #     def run(self):
    #         while True:
    #             time.sleep(self.interval)
    #             self.action()

    def __init__(self, loop_delay: float = 0,
                 configuration_file: str = None,
                 log_file: bool = False,
                 verbose: bool = False):

        self._loop_delay = loop_delay
        self._configuration_file = configuration_file
        self._verbose = verbose

        # Terminal keyboard input parameters
        self._tab_pressed = False
        self._input_buffer = ""
        self.terminal_input = None

        # Set the configuration parameters
        self._get_config()

        # Initialize the AgentHub
        self.agent_hub = AgentHub(config=self.config,
                                  verbose=self._verbose)

        # Get a reference to the AgentStatus object created by the AgentHub
        self.agent_status: AgentStatus = self.agent_hub.agent_status_obj

        # Get a reference to the AgentCommandManager created by the AgentHub
        self.acm = self.agent_hub.agent_command_manager

        # Get a reference to the C3NodeManager created by AgentHub
        self.c3nm = self.agent_hub.c3_node_manager

        # Establish the timers and logic conditionals (if, while, for ...) used
        # in the agent_core_main_loop, run_c3_config_command
        # and process_c3_input
        self.establish_logic_objects()

        # Thread to run primary control logic loop functions
        self.stopFlag = threading.Event()
        thread = threading.Thread(target=self._run_agent_core_loop_thread,
                                  args=(self.stopFlag, self._loop_delay))
        thread.start()

    def _get_config(self):
        '''
        Sets the self.config variable which is a dictionary of the YAML
        configuration file
        '''
        parser = argparse.ArgumentParser()
        args = None
        parser.add_argument(
            '--config', type=str, help='yaml config file name')
        parser.add_argument(
            '--ip', type=str, help='IP and port of the agent - overrides yaml')
        parser.add_argument(
            '--sysid', type=int, help='sys id of the agent - overrides yaml')
        args = parser.parse_args()

        if args.config is not None:
            self._configuration_file = args.config
        try:
            self.config = AgentUtils.get_config_dict(self._configuration_file)
        except FileNotFoundError:
            print("Unable to load config file - check the file name")
            sys.exit()

        if args.ip is not None and args.sysid is not None:
            self.config['SITL_ADDRESS'] = args.ip
            self.config['SYS_ID'] = args.sysid

    def establish_logic_objects(self):
        pass

    def agent_core_main_loop(self):
        '''
        Shell function that get's overriden in my_agent with custom code
        '''
        pass

    def run_c3_config_command(self, c3_config_command: str):
        '''
        Run a custom config_command from within AgentCore.  This allows you to
        run any of your psuedo code command scripts when triggered within the
        AgentCore agent_core_main_loop.
        Call this function from within your instance of AgentCore using
        self.run_c3_config_command(command)
        '''

        command = C3NodeMessage(message=c3_config_command)
        self.process_c3_input(command)

    def process_c3_input(self, c3_message: C3NodeMessage):
        '''
        Overwrite this function in your instance of an AgentCore object
        to intercept the AgentCore receiving a C3NodeMessage.  At the end
        of this overwritten function, then come back to the this function with
        super().process_c3_input(c3_message).

        Coming back to this function will handle the message if it's:
        1) a cmd_ message
        2) config_command
        '''

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

        # print(f"cmd.msg: {type(c3_message.message)}")

        # if c3_message.message[0] == '{':
        #     return

        try:
            converted = json.loads(c3_message.message)
        except json.decoder.JSONDecodeError:
            converted = c3_message.message

        if not isinstance(converted, list):
            converted = [converted]

        for cmd in converted:

            resolved = resolve_values(self.config['COMMANDS'], cmd)
            for cmd in resolved:
                # if the agent was sent a direct cmd message
                if cmd[:3] == "cmd":
                    self._process_cmd(cmd)

                elif cmd == 'delay':
                    # print("delay 3 sec")
                    time.sleep(3)

                # else check if the c3 command in the agent's
                # custom command list
                else:
                    try:
                        config_commands = self.config["COMMANDS"]
                    except KeyError:
                        print("You do not have COMMANDS available \
                              to this agent")
                        return

                    vars = []
                    if "(" in cmd:
                        main_command = cmd.split("(")[0]
                        vars = cmd.split("(")[1][:-1].split(',')

                        var_str = ""
                        for i in range(0, len(vars)):
                            var_str = f"{var_str},var{i+1}"
                        var_str = var_str[1:]
                        cmd = f"{main_command}({var_str})"

                    if cmd in config_commands.keys():
                        try:
                            # getattr(self.acm, c3_message.message)()
                            cmd = config_commands[cmd]
                            if len(vars) != 0:
                                for i in range(0, len(vars)):
                                    cmd = cmd.replace(
                                        f"var{i+1}", vars[i])
                            self._process_config_command(cmd)
                        except AttributeError:
                            print(f"3 - "
                                  f"{self.agent_hub.agent_status_obj.agent_id}"
                                  f": \'{cmd}\' failed to "
                                  f"process config command")
                    else:
                        print(f"4 - "
                              f" {self.agent_hub.agent_status_obj.agent_id}: "
                              f"\'{cmd}\' does not exist in this "
                              f"agent's config['COMMANDS']")

    def _process_config_command(self, config_command):
        '''
        If the received command does not have a cmd prefix, it is a
        custom(pseudo) command.  Look for these commands in the Agents'
        _config.yaml file
        '''

        if isinstance(config_command, list):
            cmd_list = config_command
            for cmd in cmd_list:
                self._process_cmd(cmd)
        else:
            self._process_cmd(config_command)

    def _process_cmd(self, command):
        '''
        If the received command starts with cmd, then it should be in the
        AgentCommandManager (acm) python scripts to include user defined cmd_
        '''

        def safe_eval(value):
            if value is None:
                return None
            elif isinstance(value, str) and value.lower() == 'none':
                return None
            try:
                return ast.literal_eval(value)
            except (SyntaxError, ValueError):
                # If literal_eval fails, return the value itself
                return value

        def format_dict(s):
            s = s.replace(' ', '')
            s = s.replace('{', '{"').replace(':', '":"').\
                replace(',', '","').replace('}', '"}')
            s = re.sub(r':"-?\d+(\.\d+)?"',
                       lambda m: ':' + m.group(0)[2:-1], s)
            d = json.loads(s)
            # formatted_dict = json.dumps(d)
            return d

        cmd = command.split("(")[0]
        if ("(" in command and
                "()" not in command):
            params_str = command.split("(")[1][:-1]

            # Split the string into parameters and strip whitespace
            params = [x.strip() for x in params_str.split(',')]

            # Convert parameters to appropriate types
            typed_params = []
            kwargs = {}

            # Special case for cmd_msn_load_waypoints
            if cmd == 'cmd_msn_load_waypoints':
                try:
                    # The entire parameter string should be a valid Python list
                    params_list = params_str.split('},')
                    params_list = [s + '}' for s in params_list]
                    params_list[-1] = params_list[-1][:-1]
                    waypoints = [format_dict(s) for s in params_list]
                    if not isinstance(waypoints, list):
                        raise ValueError("Expected a list of dictionaries")
                    kwargs['waypoints'] = waypoints
                except Exception as e:
                    print(f"Failed to parse waypoints: {e}")
                    return
            else:
                for param in params:
                    # Split parameter into key and value for keyword arguments
                    key_value = param.split('=')

                    if len(key_value) == 2:
                        # It's a keyword argument
                        key, value = key_value
                        val = value.strip()
                        if val == 'true':
                            val = 'True'
                        if val == 'false':
                            val = 'False'
                        if val == 'none' or val == 'None':
                            val = 'None'
                        try:
                            # kwargs[key.strip()] = ast.literal_eval(val)
                            evaled_val = safe_eval(val)
                            kwargs[key.strip()] = evaled_val
                        except ValueError:
                            print("There is something wrong with the value "
                                  "of your attributes")
                            return
                    else:
                        # It's a positional argument
                        try:
                            # Try to evaluate the parameter as a
                            # Python literal (e.g., True, 10)
                            typed_params.append(ast.literal_eval(param))
                        except ValueError:
                            # If evaluation fails, it's a string
                            typed_params.append(param)
            getattr(self.acm, cmd)(*typed_params, **kwargs)
        else:
            try:
                getattr(self.acm, cmd)()
            except AttributeError:
                print(f"2 - {self.agent_hub.agent_status_obj.agent_id}: "
                      f"\'{cmd}\' does not exist in this "
                      f"agent's AgentCommmandManager")

    # ###############################################################

    # Main agent loop
    def _run_agent_core_loop_thread(self, event, loop_delay):
        # Let the agent_status_obj and agent_hub get settled
        # with a 3 sec sleep before starting the command loop
        time.sleep(3)
        # This defines the main_loop.  It does 2 things:
        # 1) manages the keyboard inputs
        # 2) calls the agent_core_loop_functions which is overriden
        #    in the my_agent code.

        while True:
            # self._manage_keyboard_inputs()

            # Add a loop delay if needed
            # (unlikely though and the defaultis no delay)
            if not event.wait(loop_delay):

                # Run your my_agent loop functions
                # If you return a False, then stop the loop
                if not self.agent_core_main_loop():
                    break

                # Look at each C3 node and see if there is a new message
                # disregarding the message if this agent sent it
                for c3Node in self.c3nm.c3Nodes:
                    if (c3Node.is_new and
                            str(c3Node.message.sender) !=
                            str(self.config['AGENT_ID'])):
                        self.process_c3_input(c3Node.message)
