'''
'''
# import copy
import os
# import math
# import time
# import json
# from typing import Union
# from typing import List
# from typing import Optional, Tuple
from pymavlink import mavutil

from classes.commands.default_commands import DefaultCommands

os.environ['MAVLINK20'] = '1'


class DefaultMsnCommands(DefaultCommands):
    '''
    Defines waypoint (wpt._MissionWaypoint) classes and commands
    to use in AUTO mode

    Waypoints (.wpt):
        - `Waypoint`: standard waypoint
        - `SplineWaypoint`: spline waypoint
        - `CircleTurns`: orbit around a point for a # of times and at a radius
        - `Takeoff`: takoff to altitude
        - `Land`: land at current point or desired lat/lon

    Methods:
        - `cmd_msn_upload_waypoints`: upload a mission's waypoints as a list
        - `cmd_msn_start_mission`: start the uploaded mission
    '''
    # #################### Methods ###################

    def __init__(self,
                 config: dict,
                 agent_hub,
                 mav_connection,
                 verbose: bool = False):
        '''
        This is the constructor for AgentCommandManager
        '''
        super().__init__(config, agent_hub, mav_connection, verbose)

    def cmd_msn_start_mission(self,
                              verify: bool = False
                              ) -> bool:
        '''
        Start's the mission.  This assumes a mission is loaded

        Args: None
        Return: None
        '''
        self.cmd_sys_mode_change("AUTO")
        return self.send_long_command(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0, verify
        )

    def cmd_msn_clear_mission(self,
                              verify: bool = False
                              ) -> bool:
        '''
        Clears out the current mission installed on the drone
        '''
        self._mav_connection.mav.mission_clear_all_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component
        )

    def cmd_msn_load_waypoints(self,
                               waypoints: list[dict],
                               verify: bool = False) -> bool:
        '''
        Load waypoints in the form of a list of dictionaries
        '''

        msn_list = self.msn_create_waypoint_list(waypoints)
        if self.send_int_mission_items(msn_list):
            return True

    def msn_create_waypoint_list(self,
                                 waypoints: list[dict]):

        '''
        Generates a mission plan using waypoint styles from the wpt class.
        The list is a list of dictionaries where the keys of the dictionaries
        are the input parameters of the waypoint.  The dictionary must also
        include the key 'wpt_type' to classify which type of waypoint
        you are adding.

        Example usage:
        msn_list = creage_mission_waypoint_list(
            [
                {'wpt_type': 'Takeoff', 'alt': 10},
                {'wpt_type': 'Waypoint', 'lat': 38.0,
                    'lon': -104.0, 'alt': 15, 'delay': 0},
                {'wpt_type': 'SplineWaypoint', 'lat': 37.9,
                    'lon': -104.1, 'alt': 15},
                {'wpt_type': 'Land'}
            ]
        )

        '''
        mission_list = []

        for waypoint in waypoints:
            # Waypoint, SplineWaypoint, CircleTurns, ChangeSpeed, Takeoff, Land
            wpt_type = waypoint.pop('wpt_type')
            wpt_class = getattr(self.wpt, wpt_type)
            mission_list.append(wpt_class.create_waypoint(**waypoint))

        return mission_list

    def msn_append_waypoint_to_list(self,
                                    mission_list,
                                    waypoint,
                                    index: int = None
                                    ):

        if index is None:
            index = len(mission_list)

        wpt_type = waypoint.pop('wpt_type')
        wpt_class = getattr(self.wpt, wpt_type)
        mission_list.insert(index, wpt_class.create_waypoint(**waypoint))
