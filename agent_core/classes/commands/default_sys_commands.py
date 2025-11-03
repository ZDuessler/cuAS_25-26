'''
    Defines the Agent Command Manager class.  Contains the following commands:
'''

import os
# import json
from typing import Union
from pymavlink import mavutil

from classes.commands.default_commands import DefaultCommands

os.environ['MAVLINK20'] = '1'


class DefaultSysCommands(DefaultCommands):

    '''
    This class is where the commands from the UI are translated into
    Mavlink commands and sent to the Ardupilot through the
    mavlink_manager's mav_connection

    Methods:
        - `cmd_sys_mode_change()`: change flight mode
        - `cmd_sys_arm_disarm()`: arm or disarm agent
    '''

    def __init__(self,
                 config: dict,
                 agent_hub,
                 mav_connection,
                 verbose: bool = False):
        '''
        This is the constructor for AgentCommandManager
        '''
        super().__init__(config, agent_hub, mav_connection, verbose)

    def __mode_map(self, mode_name: str) -> int:
        '''
        This converts a mode string to the integer representation.
        'GUIDED' -> 4, 'POSHOLD' -> 16, etc.

        Args:
            `mode_name (str)`: name of mode (GUIDED, POSHOLD, etc.)

        Returns:
            `mode (int)`: mapped int of the mode string name
        '''
        try:
            mode = self._mav_connection.mode_mapping()[mode_name.upper()]
            return mode
        except Exception as e:
            print(f"mode selection error: {e} - canceling command")
            return None

    # ########################## Convert MAVLink Commands #####################

    def cmd_sys_mode_change(self, mode_id: Union[int, str],
                            verify=False) -> bool:
        '''
        This sends the mode_id (int) to the ArduPilot to change it's mode

        #### Params:
            `mode_id (int)`: mode_id in int to change to
                or
            `mode_id (str)`: mode_id in string to change to
            `verify (bool)`: if True, it will print a message
                if the command failed
        '''

        if isinstance(mode_id, str):
            mode_id = self.__mode_map(mode_id)
            if mode_id is None:
                return

        return self.send_long_command(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                209,
                mode_id,
                0, 0, 0, 0, 0, verify
        )

    def cmd_sys_arm_disarm(self, arm_disarm: str,
                           override=False, delay=10,
                           verify=False) -> bool:
        '''
        This sends an arm or disarm command to the ArduPilot

        Args:
            `arm_disarm (str)`: accepts 'arm' or 'disarm'
            `override (bool)`: CAUTION - overrides safety checks
            `delay (int)`: time in seconds that the arm remains
                active before it disarms if the throttles are
                not pushed up
        '''

        self._vprint("I should be setting " + arm_disarm)

        if arm_disarm == 'arm':
            arm_disarm_cmd = 1
        elif arm_disarm == 'disarm':
            arm_disarm_cmd = 0

        if override:
            override_code = 21196
        else:
            override_code = 0
        self._mav_connection.mav.param_set_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            b'DISARM_DELAY', float(delay),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        return self.send_long_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_disarm_cmd,
            override_code,
            0, 0, 0, 0, 0, verify
        )
