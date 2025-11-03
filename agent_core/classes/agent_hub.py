import subprocess
import threading
import serial.tools.list_ports
import time
import platform
import sys

from agent_status_class import AgentStatus
from classes.mavlink_manager import MavlinkManager
from classes.agent_command_manager import AgentCommandManager
from classes.c3_node import AgentC3NodeManager
from classes.agentutils import TimeKeeper


class AgentHub():

    def __init__(self, config: dict, verbose=False):

        self.config = config
        self.agent_status_obj = AgentStatus(self.config)
        self._verbose = verbose
        self.mavproxy_process = None
        self.shutdown_event = threading.Event()
        self.check_os()

        # Start MAVProxy in a separate thread
        # self.kill_process_by_name('mavproxy.py')
        mavproxy_thread = threading.Thread(target=self.start_mavproxy,
                                           daemon=True)
        mavproxy_thread.start()

        port = 14551
        while self.is_udp_port_active(port):
            print("Waiting for mavproxy to become active...")
            time.sleep(1)

        time.sleep(1)

        # Create the agent's MAVLink Manager that is
        # used to manage the messages between the instance
        # of ArduPilot (like a CUBE) and AgentCore
        self.mavlink_lost_comm = False
        self.mavlink_manager = MavlinkManager(
            self.config, self, verbose=self._verbose)
        self.mavlink_manager.run()

        # Create the agent's Command Manager that is
        # used to centralize all the MAVLink commands
        self.agent_command_manager = AgentCommandManager(
            self.config,
            self,
            self.mavlink_manager.mav_connection,
            verbose=verbose
        )

        # Create the agent's C3 Node Manager.  The manager
        # creates, stores and manages all active C3 nodes
        # as defined by the YAML file
        self.c3_node_manager = AgentC3NodeManager(self.config)

        self.send_status_updates_to_c3node()

    def start_mavproxy(self):
        """
        Function to start MAVProxy as a subprocess on Windows or Linux.
        This instance of MAVProxy will first look to see if there is a
        USB port connected to a hardware instance of ArduPilot (i.e. a Cube
        or other flight controller).  If so, it will connect to it, otherwise
        it will look to connect to the address where it expects to see an
        address for a SITL as defined in the SITL_ADDRESS parameter in
        the agent's config YAML file.
        """

        if self.check_for_screen():
            return

        # Check if this is a Linux or PC system and define the location of
        # MAVProxy based on the operating system
        # print(f"The OS is {self.config['OS']}")
        if self.config['OS'] == "LINUX":
            mp = "mavproxy.py"
        elif self.config['OS'] == "PC":
            mp = "mavproxy.exe"
        else:
            print("You are on an unknown system. "
                  "Agent Core does not support this system.")
            time.sleep(1)
            sys.exit("Exiting the program")

        # Read all the ports on the device and look for any that would
        # Indicate a hardware instance of ArduPilot is connected
        port_found = False
        ports = list(serial.tools.list_ports.comports())
        self._vprint(ports)

        # outs = f"--out=udpin:127.0.0.1:{14550 + self.config['SYS_ID']} "
        outs = "--out=udpin:127.0.0.1:14551 "
        for out in self.config['MAVPROXY_OUTPUTS']:
            outs = f"{outs} --out=udpin:0.0.0.0:{out} "
        # print(f"Outs: {outs}")

        for p in ports:
            self._vprint(p)
            if any(flt_ctlr in p.description for flt_ctlr in [
                "CubeBlack",
                "Cube",
                "CUBE",
                "USB Serial Device",
                "ArduPilot"
            ]):
                print(f"#### {self.config['AGENT_ID']} "
                      f"Establishing Connection to Cube ####")
                port_found = True
                mavproxy_command = (
                    f"{mp} "  # Use 'mavproxy.exe' on Windows
                    f"{outs}"
                    "--daemon "
                    "--non-interactive "
                    "--streamrate=-1 "
                )

        # If no ArduPilot instance was found on a hardware port then
        # connect MAVProxy to the IP:port combo defined by SITL_ADDRESS
        if not port_found:
            print(f"#### {self.config['AGENT_ID']} "
                  f"Establishing Connection to SITL ####")
            mavproxy_command = (
                f"{mp} "  # Use 'mavproxy.exe' on Windows
                f"--master={self.config['SITL_ADDRESS']} "
                f"{outs}"
                "--daemon "
                "--non-interactive "
                "--streamrate=-1"
            )
        # Run MAVProxy as a subprocess and don't display the information
        # print(mavproxy_command)

        if self.config["OS"] == "PC":
            # Use 'start' to open a new window and
            # 'min' to minimize it on Windows
            subprocess.Popen(
                # test,
                f'start /min cmd /c "{mavproxy_command}"',
                shell=True,
                # stdout=subprocess.DEVNULL,
                # stderr=subprocess.DEVNULL
            )
        elif self.config["OS"] == "LINUX":
            session_name = f'mavproxy_{self.config["AGENT_ID"]}'
            subprocess.run(['screen', '-dmS', session_name])
            subprocess.Popen(['screen', '-S', session_name, '-p', '0',
                              '-X', 'stuff', f'{mavproxy_command}\n'])

    def send_status_updates_to_c3node(self):

        def send_status_message(c3node_name, status_msg_id):

            status_msg_id = status_msg_id.lower()
            status_obj = getattr(self.agent_status_obj, status_msg_id)
            status_dict = {status_msg_id: status_obj.to_dict()}
            c3node = getattr(self.c3_node_manager, c3node_name)
            c3node.send_direct_message(status_dict)

        for c3node in self.config['C3_NODES']:
            direct = self.config['C3_NODES'][c3node]['DIRECT']
            if 'STATUS_HEARTBEATS' in direct:
                status_heartbeat_list = direct['STATUS_HEARTBEATS']
                # print(status_heartbeat_list)
                for heartbeat in status_heartbeat_list:
                    status_key = list(heartbeat.keys())[0]
                    status_value = list(heartbeat.values())[0]
                    TimeKeeper(status_value, lambda c=c3node, k=status_key:
                               send_status_message(c, k))

    def check_os(self):
        os_name = platform.system()
        # print(f"OS: {os_name}")
        if str(os_name) == "Windows":
            # print("You are on a Windows PC.")
            self.config["OS"] = "PC"
        elif str(os_name) == "Linux":
            # print("You are on a Linux system.")
            self.config["OS"] = "LINUX"
        else:
            print(f"You are on an unknown system: {os_name}. "
                  f"Agent Core does not support this system.")
            self.config["OS"] = "UNKNOWN"
            time.sleep(1)
            sys.exit("Exiting the program")

    def is_udp_port_active(self, port):
        result = subprocess.run(['netstat', '-an'],
                                capture_output=True, text=True)
        for line in result.stdout.splitlines():
            if f"127.0.0.1:{port}" in line:
                return False
        return True

    def kill_mavproxy_session(self):
        try:
            print(f"Killing the session mavproxy_{self.config['AGENT_ID']}")
            session_name = f'mavproxy_{self.config["AGENT_ID"]}'
            subprocess.run(f'screen -S {session_name} -X kill')
        except FileNotFoundError:
            pass

    def check_for_screen(self):
        try:
            # Check if the screen session is active
            result = subprocess.run(['screen', '-ls'],
                                    capture_output=True, text=True)
            sessions = result.stdout.splitlines()
            # Loop through each session and kill matching the session_name
            for session in sessions:
                if f"mavproxy_{self.config['AGENT_ID']}" in session:
                    # Extract the unique session identifier (number)
                    session_id = session.split()[0].strip()
                    return session_id
        except Exception as e:
            print(f"An error occurred: {e}")
            return None
        return None

    def _vprint(self, print_string):
        if self._verbose:
            print(print_string)
