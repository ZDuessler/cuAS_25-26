'''
    Defines the Agent Command Manager class.  Contains the following commands:

    Most pymavlink NAV commands are [COMMANDS] and are executed within either:
    COMMAND_INT (#75) - self._mav_connection.mav.command_int_send()
    COMMAND_LONG (#76) - self._mav_connection.mav.command_long_send()

    .command_int_send() is used for passing lat/lon commands as ints
        allows for higher precision than longs.
    .command_long_send() are used for all other commands

    However, some commands don't/can't follow the 7-parameter format of
    INT and LONG and are still considered [MESSAGES] vs. [COMMANDS] These are
    built into pymavlink with their own _send() functions.  If a [MESSAGE] does
    not begin with 'SET_', then you can call the [MESSAGE]
    with the _send version.

    For example: set_position_target_global_int_send() implements the
    SET_POSITION_TARGET_GLOBAL_INT ( #86 ) message.  As a rule of thumb,
    if a [MESSAGE] starts with 'SET_' it likely has mav function such as:
    SET_POSITION_TARGET_GLOBAL_INT -> mav.set_position_target_global_int_send()

    Additionally, some [MESSAGE]s have a _send option but are not intended
    to set/send information.  Instead, use the .mav.command_long_send() with
    MAV_CMD_REQUEST_MESSAGE to request the information

    In all cases, you can implement a [MESSAGE] using a more
    generic .mav.send() function.  With this function, you need to call the
    name of the MAVLink function (such as SET_POSITION_TARGET_GLOBAL_INT which
    is defined as 'MAVLink_set_position_target_global_int_message') and pass
    the parameters. The best way to do that is to follow the format below:

        function_name = 'MAVLink_set_position_target_global_int_message'
        message_function = getattr(mavutil.mavlink, msg_type)
        self._mav_connection.mav.send(
            message_function(.... required parameters ....)
'''

import os
import math
# import time
# import json
from typing import Optional, Tuple
from pymavlink import mavutil

from classes.commands.default_commands import DefaultCommands
import classes.agentutils as agentutils


os.environ['MAVLINK20'] = '1'


class DefaultNavCommands(DefaultCommands):

    '''
    This class is where the GUIDED navigation commands from the
    UI are translated into Mavlink commands and sent to the Ardupilot
    through the mvlink_manager's mav_connection

    ### Methods:

    ##### Verifiable Methods (methods that return T/F if successfully accepted)
        - `cmd_nav_guided_takeoff()`: takeoff to set height
        - `cmd_nav_guided_land()`: land at present position
        - `cmd_nav_guided_set_speed()`: set speed while in GUIDED
        - `cmd_nav_guided_reposition_hat()`: move to new lat/lon/alt(hat)
        - `cmd_nav_guided_reposition_msl()`: move to new lat/lon/alt(msl)
        - `cmd_nav_guided_goto_alt_hat()`: goto height above takeoff altitude
        - `cmd_nav_guided_goto_alt_msl()`: goto msl altitude
        - `cmd_nav_guided_set_yaw()`: set the yaw (hdg or relative)
        - `cmd_nav_guided_slew_br_hat()`: set the bearing, range alt(hat)
        - `cmd_nav_guided_slew_br_msl()`: set the bearing, range alt(msl)
    ##### Non-Verifiable Methods (methods that don't verify send/accept stats)
        - `cmd_nav_guided_goto_pos_hat()`: goto pos & height above takeoff
        - `cmd_nav_guided_goto_pos_msl()`: goto pos & height msl
        - `cmd_nav_guided_ned_velocity()`: set North/East/Down velocity
        - `cmd_nav_guided_track_velocity()`: set track velocity
        - `cmd_nav_guided_xyz_velocity()`: set body-xyz velocity
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

    # ####### Verifiable GUIDED Commands ########

    def cmd_nav_guided_takeoff(self,
                               height=10,
                               verify=False
                               ) -> bool:
        '''
        Sends a takeoff command to the Ardupilot to a specific height

        #### Params
            `height (int)`: height in meters above the takeoff point
              that the Ardupilot will climb to

        #### Return:
            `bool`: verify
        '''

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            return self.send_long_command(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0, int(height),
                verify)
        else:
            return False

    def cmd_nav_guided_land(self,
                            xyz: Optional[Tuple[float, float, float]] = None,
                            latlon: Optional[Tuple[float, float]] = None,
                            verify=False
                            ) -> bool:
        '''
        Sends a land command to the Ardupilot.  No passed arguments will
        land the ArduPilot at it's current location.

        #### Params:
            `xyz (tuple(float, float, float))`:
                relative position in meters from the home location
            `latlon (tuple(float, float))`: lat lon position to land

        #### Return:
            `bool`: verify
        '''

        if xyz is not None:
            landing_location = xyz
        elif latlon is not None:
            landing_location = (latlon[0], latlon[1], 0)
        else:
            landing_location = (0, 0, 0)

        return self.send_long_command(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            landing_location[0],
            landing_location[1],
            landing_location[2],
            verify
        )

        # self._mav_connection.mav.command_long_send(
        #     self._mav_connection.target_system,
        #     self._mav_connection.target_component,
        #     mavutil.mavlink.MAV_CMD_NAV_LAND,
        #     0,  # Confirmation
        #     0,  # Empty
        #     0,  # Empty
        #     0,  # Empty
        #     0,  # Yaw Angle
        #     landing_location[0],
        #     landing_location[1],
        #     landing_location[2],
        # )

    def cmd_nav_guided_set_speed(self,
                                 speed: float = 5,
                                 speedType: str = 'GROUND',
                                 verify: bool = False
                                 ) -> bool:
        '''
        Set the speed the copter flies while in guided mode.

        #### Params:
            `speed (float)`: speed in the forward direction (m/s)
            `speetType (str)`: type of speed - 'AIR' or 'GROUND'
             Default is to 'GROUND'

        #### Return:
            `bool`: verify
        '''
        if self._mav_connection is not None:
            if speedType is None:
                speedType = 'GROUND'
            if speedType.upper() == 'AIR':
                speed_type = 0  # 0 for airspeed
            else:
                speed_type = 1  # if not airspeed, default 1 to ground speed

        return self.send_long_command(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            speed_type,
            speed,
            -1,
            0, 0, 0, 0,
            verify
        )

    def cmd_nav_guided_reposition_hat(self,
                                      lat: float = None,
                                      lon: float = None,
                                      alt: float = None,
                                      verify: bool = False) -> bool:

        '''
        Sends a simple command to goto a lat/lon and HAT alt.
        It will go at the speed that the drone is current set to.
        If you set lat or lon to None then the drone will slew East-West
        (with lat set to None) and North-South (with lon set to None)

        #### Params:
            `lat (float)`: the desired lat location
            `lon (float)`: the desired lon location
            `alt (float)`: altitude in height above takeff(m) to fly to. If alt
                is None this will send the drone to the location at the drone's
                current altitude.

        #### Return:
            `bool`: verify
        '''

        if self.cmd_sys_mode_change('GUIDED', True):

            if lat is None:
                lat_int = int(self._agent_hub.
                              agent_status_obj.agent_position.lat * 1e7)
            else:
                lat_int = int(lat*1e7)
            if lon is None:
                lon_int = int(self._agent_hub.
                              agent_status_obj.agent_position.lon * 1e7)
            else:
                lon_int = int(lon*1e7)

            if alt is None:
                alt = self._agent_hub.agent_status_obj\
                    .agent_position.relative_alt
            else:
                alt = int(alt)

            return self.send_int_command(
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, 0, 0, 0, 0, lat_int, lon_int, alt, verify
            )
        else:
            return False

    def cmd_nav_guided_reposition_msl(self,
                                      lat: float = None,
                                      lon: float = None,
                                      alt: float = None,
                                      verify: bool = False
                                      ) -> bool:

        '''
        Sends a simple command to goto a lat/lon and msl alt.
        It will go at the speed that the
        drone is current set to.  If you set lat or lon to None
        then the drone will slew East-West (with lat set to None)
        and North-South (with lon set to None)

        #### Params:
            `lat (float)`: the desired lat location
            `lon (float)`: the desired lon location
            `alt (float)`: altitude in MSL (m) to fly to.  If alt is None
                this will send the drone to the location at the drone's
                current altitude.

        #### Return:
            `bool`: verify
        '''

        if self.cmd_sys_mode_change('GUIDED', True):

            if lat is None:
                lat_int = int(self._agent_hub.
                              agent_status_obj.agent_position.lat * 1e7)
            else:
                lat_int = int(lat*1e7)
            if lon is None:
                lon_int = int(self._agent_hub.
                              agent_status_obj.agent_position.lon * 1e7)
            else:
                lon_int = int(lon*1e7)

            if alt is None:
                alt = self._agent_hub.agent_status_obj\
                    .agent_position.relative_alt
            else:
                alt = int(alt)

            return self.send_int_command(
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, 0, 0, 0, 0, lat_int, lon_int, alt, verify
            )

    def cmd_nav_guided_goto_alt_hat(self,
                                    alt: float,
                                    verify: bool = False
                                    ) -> bool:
        '''
        Send the copter to the selected height above takeoff

        #### Params
            `alt (float)`: height above takeoff elevation

        #### Return:
            `bool`: verify
        '''
        if self.cmd_sys_mode_change('GUIDED', verify=True):
            agent_pos = self._agent_hub.agent_status_obj.agent_position
            # self.cmd_nav_guided_goto_pos_hat(
            #     agent_pos.lat, agent_pos.lon, alt)
            return self.cmd_nav_guided_reposition_hat(agent_pos.lat,
                                                      agent_pos.lon,
                                                      alt, verify=verify)
        else:
            return False

    def cmd_nav_guided_goto_alt_msl(self,
                                    alt: float,
                                    verify: bool = False
                                    ) -> bool:
        '''
        Send the copter to the selected MSL altitude

        #### Params
            `alt (float)`: MSL altitude

        #### Return:
            `bool`: verify
        '''
        if self.cmd_sys_mode_change('GUIDED', verify=True):
            agent_pos = self._agent_hub.agent_status_obj.agent_position
            # self.cmd_nav_guided_goto_pos_hat(
            #     agent_pos.lat, agent_pos.lon, alt)
            return self.cmd_nav_guided_reposition_msl(agent_pos.lat,
                                                      agent_pos.lon,
                                                      alt, verify=verify)
        else:
            return False

    def cmd_nav_guided_set_yaw(self,
                               yaw: float,
                               yawRate: float = 0,
                               relative: bool = False,
                               CW: int = 0,
                               verify=True
                               ) -> bool:
        '''
        Sets the yaw value while in guided mode.  This allows you update which
        way the drone is facing while still moving to a commanded point in
        GUIDED mode.

        #### Params:
            `yaw (float)`: The heading or relative offset angle the drone
             should face (deg)
            `yawRate (float)`: The rate of change in commanded yaw (deg/sec)
            `relative (bool)`: Sets whether the yaw is relative to the forward
             direction (True) or in absolute heading (False)
             `CW (int)`: -1 = CCW, 0 - closest direction, 1 = CW

        #### Return:
            `bool`: verify
        '''

        if relative:
            _relative = 1
        else:
            _relative = 0

        # yaw = math.radians(yaw)
        # yawRate = math.radians(yawRate)
        if yaw < 0:
            yaw = 360 + yaw
        yaw = yaw % 360

        return self.send_long_command(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,          # confirmation
            yaw,        # yaw
            yawRate,    # yaw rate
            CW,         # 0: closest, -1: CCW, 1: CW
            _relative,  # is yaw relative to current yaw or absolute hdg
            0, 0, 0,    # params 5-7 (not used, set to 0)
            verify
        )

    def cmd_nav_guided_slew_br_hat(self,
                                   bearing,
                                   range,
                                   alt=None,
                                   hdg=None,
                                   relative=False,
                                   ) -> bool:

        '''
        Sends the drone to a lat/lon/alt defined by a bearing
        (cardinal heading) and range (meters) from the current
        location of the drone.

        #### Params:
            `bearing (deg)`: the cardinal direction to travel in degrees
            `range (meters)`: the distance to travel
            `alt (meters)`: the height above takeoff (optional)
            `hdg (deg)`: the direction the drone should point in
                a cardinal heading (optional)
            `relative (bool)`: point the nose of the drone in an angle (hdg)
                relative to the direction of travel (optional)

        #### Return:
            `bool`: verify
        '''

        lat = (self._agent_hub.
               agent_status_obj.agent_position.lat)
        lon = (self._agent_hub.
               agent_status_obj.agent_position.lon)
        dest_lat, dest_lon = (agentutils.AgentUtils.
                              calculate_destination_point_light)(
            lat, lon, bearing, range
        )

        if alt is None:
            alt = (self._agent_hub.
                   agent_status_obj.agent_position.relative_alt)

        if hdg is None:
            hdg = (self._agent_hub.
                   agent_status_obj.agent_position.hdg)
        elif hdg is not None and relative:
            hdg = bearing + hdg

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            pos_return = self.cmd_nav_guided_reposition_hat(
                lat=dest_lat,
                lon=dest_lon,
                alt=alt
            )
            hdg_return = self.cmd_nav_guided_set_yaw(hdg, relative=False)
            if pos_return and hdg_return:
                return True
            else:
                return False
        else:
            return False

    def cmd_nav_guided_slew_br_msl(self,
                                   bearing,
                                   range,
                                   alt=None,
                                   hdg=None,
                                   relative=False,
                                   ) -> bool:

        '''
        Sends the drone to a lat/lon/alt defined by a bearing
        (cardinal heading) and range (meters) from the current
        location of the drone.

        #### Params:
            `bearing (deg)`: the cardinal direction to travel in degrees
            `range (meters)`: the distance to travel
            `alt (meters)`: the MSL altitude to fly to (optional).  If None,
                the drone will fly at the current altitude
            `hdg (deg)`: the direction the drone should point in.
                a cardinal heading (optional).  If None, the drone will face
                foward as it flies
            `relative (bool)`: point the nose of the drone in an angle (hdg)
                relative to the direction of travel (optional)

        #### Return:
            `bool`: verify
        '''

        lat = (self._agent_hub.
               agent_status_obj.agent_position.lat)
        lon = (self._agent_hub.
               agent_status_obj.agent_position.lon)
        dest_lat, dest_lon = (agentutils.AgentUtils.
                              calculate_destination_point_light)(
            lat, lon, bearing, range
        )

        if alt is None:
            alt = (self._agent_hub.
                   agent_status_obj.agent_position.relative_alt)

        if hdg is None:
            hdg = (self._agent_hub.
                   agent_status_obj.agent_position.hdg)
        elif hdg is not None and relative:
            hdg = bearing + hdg

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            pos_return = self.cmd_nav_guided_reposition_msl(
                lat=dest_lat,
                lon=dest_lon,
                alt=alt
            )
            hdg_return = self.cmd_nav_guided_set_yaw(hdg, relative=False)
            if pos_return and hdg_return:
                return True
            else:
                return False
        else:
            return False

    # ### Non-verifiable but more complex commands ###
    def cmd_nav_guided_goto_pos_hat(self,
                                    lat: float = None,
                                    lon: float = None,
                                    alt: float = None,
                                    hdg: float = None,
                                    relative_hdg: bool = False,
                                    yawRate: float = None,
                                    speed: float = None,
                                    speedType: str = None,
                                    pos_dict: dict = None,
                                    ):
        """
        Sends a command to go to a specific latitude, longitude, altitude
        and yaw angle.  Yaw angle places the nose of the
        aircraft at that mag heading.
        The altitude is height above takeoff (HAT) in meters (m).

        #### Params:
            `lat (float)`: The target latitude
            `lon (float)`: The target longitude
            `alt (float, optional)`: Height above takeoff (m).
                If you don't set `alt`, it will fly to the point
                at the current altitude.
            `hdg (float, optional)`: The target hdg (deg) in NED.
                Defaults to forward.
            `relative_hdg (bool)`: hdg is angle off of forward direction
            `yawRate (float, optional)`: Target yaw rate (deg/sec).
                Stops yawing once you hit the desired rate.
            `speed (float, optional)`: The forward speed of the drone in m/s
                If speed is not set, it will command max airspeed
            `speedType (str, optional)`: "AIR" or "GROUND" speed

        #### Returns:
            None
        """

        if alt is None:
            alt = self._agent_hub.agent_status_obj\
                .agent_position.relative_alt

        if relative_hdg:
            hdg = self._agent_hub.agent_status_obj\
                .agent_position.hdg + hdg

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            self.set_position_target(lat=lat, lon=lon,
                                     relative_alt=alt, hdg=hdg,
                                     yawRate=yawRate,
                                     )
            if speed is not None:
                self.cmd_nav_guided_set_speed(speed, speedType)
            else:
                self.cmd_nav_guided_set_speed(100.0, "AIR")

    def cmd_nav_guided_goto_pos_msl(self,
                                    lat: float,
                                    lon: float,
                                    alt: float = None,
                                    hdg: float = None,
                                    relative_hdg: bool = False,
                                    yawRate: float = None,
                                    speed: float = None,
                                    speedType: str = None):
        """
        Sends a command to go to a specific latitude, longitude, altitude
        and yaw angle.  Yaw angle places the nose of the
        aircraft at that mag heading.
        The altitude is height above takeoff (HAT) in meters (m).

        #### Params:
            `lat (float)`: The target latitude
            `lon (float)`: The target longitude
            `alt (float, optional)`: Height above sea level (m).
                If you don't set `alt`, it will fly to the point
                at the current altitude.
            `yaw (float, optional)`:
                The target yaw (rad) in NED. Defaults to forward.
            `yawRate (float, optional)`:
                Target yaw rate (rad/sec)
            `speed (float, optional)`:
                The forward speed of the drone in m/s.
                If speed is not set, it will command max airspeed
            `speedType (str, optional)`: "AIR" or "GROUND" speed

        #### Return:
            None
        """

        if alt is None:
            alt = self._agent_hub.agent_status_obj.agent_position.alt

        if relative_hdg:
            hdg = self._agent_hub.agent_status_obj\
                .agent_position.hdg + hdg

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            self.set_position_target(lat=lat, lon=lon, msl_alt=alt,
                                     hdg=hdg, yawRate=yawRate)
            if speed is not None:
                self.cmd_nav_set_speed(speed, speedType)
            else:
                self.cmd_nav_set_speed(100.0, "AIR")

    def cmd_nav_guided_ned_velocity(self,
                                    nVel: float,
                                    eVel: float,
                                    dVel: float,
                                    yaw: float = None,
                                    yawRate: float = None
                                    ):
        '''
        Sets the 3 velocity vectors in relation to North/East/Down.
        The yaw (hdg angle) can be set independently.

        #### Params:
            `nVel (float)`: speed North (m/s)
            `nVel (float)`: speed East (m/s)
            `nVel (float)`: speed Down (m/s)
            `yaw (float)`: absolute heading (deg). Leaving it
                blank will keep the current nose forward heading
            `yawRate (float)`" yaw rate to get to yaw (deg/sec)

        #### Return:
            None
        '''

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            if yawRate is not None:
                yawRate = math.radians(yawRate)
            self.set_position_target(velxyz=(nVel, eVel, dVel),
                                     hdg=yaw, yawRate=yawRate)

    def cmd_nav_guided_track_velocity(self,
                                      track_hdg: float,
                                      speed: float,
                                      dVel: float = 0,
                                      yaw: float = None,
                                      yawRate: float = None
                                      ):
        '''
        Sets the magnetic track direction of the copter.
        The yaw (hdg angle) can be set independently.

        #### Params:
            `track_hdg (float)`: track heading (deg)
            `speed (float)`: speed along the track vector (m/s)
            `dVel (float)`: speed of up/down (m/s) ... + is down
            `yaw (float)`: absolute heading (deg). Leaving it
                blank will keep the current nose forward heading
            `yawRate (float)`" yaw rate to get to yaw (deg/sec)

        #### Return:
            None
        '''

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            nVel = math.cos(math.radians(track_hdg))*speed
            eVel = math.sin(math.radians(track_hdg))*speed
            self.set_position_target(velxyz=(nVel, eVel, dVel),
                                     yaw=yaw, yawRate=yawRate)

    def cmd_nav_guided_xyz_velocity(self,
                                    xVel: float,
                                    yVel: float,
                                    zVel: float,
                                    yaw: float = None,
                                    yawRate: float = None
                                    ):
        '''
        Sets the 3 velocity vectors in relation to Forward/Right/Down.
        Adding a yaw (hdg) angle will rotate the copter towards that
        heading AFTER it sets the body-frame velocity vector.

        #### Params:
            `xVel (float)`: body forward velocity (m/s)
            `yVel (float)`: body right velocity (m/s)
            `zVel (float)`: body down velocity (m/s)
            `yaw (float)`: absolute heading (deg). Leaving it
                blank will keep the current nose forward heading.
            `yawRate (float)`" yaw rate to get to yaw (deg/sec)

        #### Return:
            None
        '''

        if self.cmd_sys_mode_change('GUIDED', verify=True):
            yaw_angle = self._agent_hub.agent_status_obj.agent_position.hdg_rad

            # Calculate the cosine and sine of the yaw angle
            cos_yaw = math.cos(yaw_angle)
            sin_yaw = math.sin(yaw_angle)

            # Convert body-relative velocities to NED velocities
            vx_ned = xVel * cos_yaw - yVel * sin_yaw
            vy_ned = xVel * sin_yaw + yVel * cos_yaw
            vz_ned = zVel  # Down velocity remains the same

            if yaw is None:
                yaw = yaw_angle

            self.set_position_target(velxyz=(vx_ned, vy_ned, vz_ned), yaw=yaw)
