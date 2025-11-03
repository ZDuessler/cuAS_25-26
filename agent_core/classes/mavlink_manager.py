import os
import time
from threading import Thread, Event, Lock
from datetime import datetime, timedelta

from agent_status_class import AgentStatus

from pymavlink import mavutil
os.environ['MAVLINK20'] = '1'


class MavlinkManager:
    '''
        This class is used to talk to the MavLink instance
        (SITL or ArduPilot Hardware)
    '''

    def __init__(self, config, agent_hub, verbose=False):

        # class settings and parametes
        self.config = config
        self.agent_hub = agent_hub
        self.agent_status_obj: AgentStatus = self.agent_hub.agent_status_obj
        self._verbose = verbose

        # # aurdupilot connectivity status parameters
        self.mav_connection = None
        self.port_connected = False
        self.running_in_sim = False
        self.mavlink_ready = False
        # self._time_of_last_ardupilot_heartbeat = datetime.now()
        # self.wait_for_ack = False
        self.current_mavlink_message_dict = dict()

        self.log_timer_dict = {}
        self.combined_msg_intervals = {**self.config['MESSAGE_INTERVALS'],
                                       **self.config['USER_MESSAGE_INTERVALS']}
        self.log_lock = Lock()
        self.rotate_log = Event()

        self.initialize_logging()
        self._initialize_the_port()

        # ### Sets up the listener for the Ardupilot Heartbeat
        # and other designated messages with intervals (GPS, etc.)
        self._mm_mavlink_message_thread = Thread(
            target=self.mm_message_queue)
        self._mm_mavlink_message_thread.daemon = True

    def vprint(self, print_string):
        if self._verbose:
            print(print_string)

    def run(self):
        ''' Starts the thread that listens for messages from MavProxy '''
        self._mm_mavlink_message_thread.start()

    def stop(self):
        ''' Stops the thread that listens for messages from MavProxy'''
        self._mm_mavlink_message_thread.join()

    def _initialize_the_port(self):
        ''' Gets the port the Cube is connected to '''

        def _wait_for_heartbeat():

            try:
                self.vprint("waiting for initial adupilot heartbeat")
                try:
                    self.mav_connection.mav.heartbeat_send(1, 1, 1, 1, 1, 1)
                    self.mav_connection.wait_heartbeat()
                except OSError as e:
                    print(f"Mavlink Manager Error: {e}")
                self.vprint("received intitial ardupilit heartbeat")
                self.mavlink_ready = True
                self.vprint("Heartbeat from ardupilot: " +
                            str(self.mav_connection.target_system) +
                            " / " + str(self.mav_connection.target_component))
                return True
            except TypeError:
                self.vprint("Failed to get heartbeat from ardupilot")
                return False

        def _set_message_intervals():

            req_msg_interval_dict = self.config['MESSAGE_INTERVALS']
            req_user_msg_interval_dict = self.config['USER_MESSAGE_INTERVALS']
            req_msg_interval_dict.update(
                req_user_msg_interval_dict)

            for requested_message in req_msg_interval_dict.keys():

                self.vprint(str(requested_message) +
                            " / " +
                            str(req_msg_interval_dict[requested_message]))

                self.mav_connection.mav.command_long_send(
                    self.mav_connection.target_system,
                    self.mav_connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    int(req_msg_interval_dict[requested_message]['ID']),
                    req_msg_interval_dict[requested_message]['INTERVAL'],
                    0,
                    0,
                    0,
                    0,
                    0
                )
                ackmsg = self.mav_connection.recv_match(type='COMMAND_ACK',
                                                        blocking=True)
                self.vprint(ackmsg)

        # No serial port connections are found so got simulation mode
        self.vprint(f"looking for an ardupilot connection on: "
                    # f"udpout:127.0.0.1:{14550 + self.config['SYS_ID']}")
                    f"udpout:127.0.0.1:{14551}")
        self.mav_connection = mavutil.mavlink_connection(
            # f"udpout:127.0.0.1:{14550 + self.config['SYS_ID']}")
            f"udpout:127.0.0.1:{14551}")
        self.vprint("Connected")
        self.mav_connection.mav.srcSystem = int(self.config['SYS_ID'])
        if self.mav_connection is not None:
            self.vprint("There is a connection to the ardupilot")
            self.running_in_sim = True
        else:
            self.vprint("There is not a created mav_connection")
            self.running_in_sim = False

        # Set the target_system for this vehicle for MAVLink message traffic
        self.mav_connection.target_system = self.config['SYS_ID']

        # If wait_for_heartbeat works, you are ready to start
        # talking across the mav_connection
        if _wait_for_heartbeat() is True:
            self.vprint("I heard the first heartbeat from the ardupilot")
            self.mavlink_ready = True
            _set_message_intervals()
        # else the port you've connected to is no longer valid (no heartbeat)
        # so try and reconnect to the port again
        else:
            self.port_connected = False
            self.mavlink_ready = False

    def mm_message_queue(self):

        while True:

            # If the mav_connection is not established yet,
            # then don't run the function
            if self.mav_connection.fd is None:
                return

            # Get all the desired messages in the MESSAGE_INTERVALS
            # yaml dictionary
            message_names = list(self.config['MESSAGE_INTERVALS'].keys())
            user_message_names = list(
                self.config['USER_MESSAGE_INTERVALS'].keys())

            msg_from_ardupilot = None
            msg_type = None

            # LISTEN for the desired messages from the ardupilot
            msg_from_ardupilot = self.mav_connection.recv_match(blocking=True)
            # print(f"{self.mav_connection.target_system}")
            # if (msg_from_ardupilot.get_srcSystem() ==
            #         self.mav_connection.target_system):

            if (msg_from_ardupilot.get_srcSystem() ==
                    self.config['SYS_ID']):


                msg_type = msg_from_ardupilot.get_type()
                self.current_mavlink_message_dict[msg_type] = \
                    msg_from_ardupilot

                # Lost MAVLink Logic
                # self.check_for_lost_mavlink_connection()

                # If the message is in the list of default or custom
                # messages as defined by the YAML file
                if (msg_type in message_names or
                        msg_type in user_message_names):
                    # if the agent_status_obj does not already have a class for
                    # this message then build one
                    if not hasattr(self.agent_status_obj, msg_type):
                        self.agent_status_obj.build_message_class(
                            msg_type, msg_from_ardupilot.to_dict())
                    # else a class exists for this message.
                    # Update that class with new message
                    else:
                        # if msg_type == 'HEARTBEAT':
                        #     print(f"{msg_from_ardupilot.get_srcSystem()}: "
                        #           f"{msg_from_ardupilot}")
                        self.agent_status_obj.update_message_object(
                            msg_from_ardupilot)

                # elif msg_type == "COMMAND_ACK":
                #     print(f"{self.mav_connection.target_system}: "
                #           f"{msg_from_ardupilot}")

                # log the timestamped data to the text file
                if msg_type in self.log_timer_dict:
                    if self.log_timer_dict[msg_type] < datetime.now():
                        self.log_string(str(msg_from_ardupilot.to_dict()))
                        self.log_timer_dict[msg_type] = \
                            (datetime.now() +
                                timedelta(
                                microseconds=int(
                                    self.combined_msg_intervals[msg_type]
                                    ['LOG_INTERVAL'])))

    def check_for_lost_mavlink_connection(self):
        heartbeat_time = datetime.strptime(self.agent_status_obj.timestamp,
                                           "%m/%d/%Y, %H:%M:%S.%f")
        delta_time = (datetime.now() - heartbeat_time).seconds
        if delta_time > self.config['MAVLINK_LOST_COMM_LIMIT']:
            self.agent_status_obj.flight_mode = "NO MAVLINK"
            print("AgentCore has lost come with MAVLink")

    # ##################### Manage Logging ######################
    '''
        To log data, from within your my_agent loop call:
        AgentHub.MavlinkManager.start_logging.set()
        To stop loggin data, call:
        AgentHub.MavlinkManager.start_logging.clear()

        To start a new log file, call:
        AgentHub.MavlinkManager.rotate_log.set()
    '''

    def initialize_logging(self):

        def write_to_file(string_list,
                          start_logging, log_lock,
                          rotate_log_event, log_interval):

            def rotate_log_file():
                script_directory = os.path.dirname(os.path.realpath(__file__))
                parent_parent_directory = \
                    os.path.abspath(os.path.join(
                        script_directory, os.pardir, os.pardir))
                logs_directory = \
                    os.path.join(
                        parent_parent_directory, 'logs', 'ac_flight_logs')
                os.makedirs(logs_directory, exist_ok=True)

                log_filename = os.path.join(
                    logs_directory, str(
                            datetime.now().strftime('%Y%m%d_%H%M%S') + '.txt'))
                return log_filename

            while True:
                if start_logging.is_set():
                    with log_lock:

                        if rotate_log_event.is_set():
                            rotate_log_event.clear()
                            filename = rotate_log_file()

                        if string_list[0] != "":
                            try:
                                with open(filename, 'a') as f:
                                    f.write(string_list[0])
                                string_list[0] = ""
                            except IOError as e:
                                print(e)
                                pass
                else:
                    string_list[0] = ''

                time.sleep(log_interval)

        for key, value in self.combined_msg_intervals.items():
            if value['LOG_INTERVAL'] != -1:
                if value['LOG_INTERVAL'] == 0:
                    msg_log_interval = int(value['INTERVAL'])
                else:
                    msg_log_interval = int(value['LOG_INTERVAL'])
                self.log_timer_dict[key] = \
                    (datetime.now() + timedelta(microseconds=msg_log_interval))

        self.log_str = [""]
        self.start_logging = Event()
        self.start_logging.clear()
        message_log_thread = Thread(
            target=write_to_file,
            args=(
                self.log_str,
                self.start_logging, self.log_lock,
                self.rotate_log, 1),
            daemon=True
            )
        message_log_thread.start()

    def log_string(self, string):
        if self.start_logging.is_set():
            with self.log_lock:
                self.log_str[0] = str(
                    self.log_str[0] +
                    str(self.agent_status_obj.timestamp) +
                    " --- " +
                    str(string + "\n"))
