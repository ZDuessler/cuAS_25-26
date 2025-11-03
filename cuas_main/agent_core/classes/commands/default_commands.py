
import threading
import time
import math
import copy
from typing import Optional, Tuple, List, Union
from pymavlink import mavutil


class DefaultCommands:

    def __init__(self,
                 config: dict,
                 agent_hub,
                 mav_connection,
                 verbose: bool = False):
        '''
        This is the constructor for creating catagorized command classes
        such as sys, nav and msn.  This allows inheriting classes to
        more simply call and confirm receipt of MAVLINK-structured messages
        from within a deamon thread so as to not stall the main loop

        Args:
            - `config (dict)`: agent_configuration.yaml
            - `agent_hub (AgentHub)`: parent agent_hub
            - `mav_connection(mavutil.mavlink_connection)`:
                reference to the MavlinkManager's mav_connection
            - `verbose (bool)`: show expanded info
        '''
        self.config = config
        self._agent_hub = agent_hub
        self._mav_connection = mav_connection
        self._retry_count = 0
        self._lock = threading.Lock()  # Lock for thread safety
        self._verbose = verbose

    def _ack(self, keyword):
        '''
        Used to listen for return messages after data uploads.

        Args:
            `keyword (str)`: the name of the message you are waiting
            to hear from

        Return: None
        '''
        timeout = 10
        msg = self._mav_connection.recv_match(
            type=keyword, blocking=True, timeout=timeout)
        if False:
            print(f"-- Message Read "
                  f"{str(msg)}")

    # ### Default Command Classes ###

    class wpt:

        class _MissionWaypoint:
            '''
            Default Mission Waypoint class.
            Used as a baseline for other Waypoint classes.

            Waypoint Structure:
                1. Sequence: This is the order of the waypoints. It starts
                    from 0.
                2. Current WP: This is always set to 0 in mission files,
                    and the vehicle will set it to 1 for the current waypoint.
                3. Frame: This defines the coordinate system. 0 is for
                    a global coordinate system, 3 is for a local coordinate
                    system.
                4. Command: This is the action for the waypoint. For example,
                    16 is for a simple waypoint, 22 is for taking off, and 178
                    is for changing speed.
                5. Param1: This is the first parameter and its meaning depends
                    on the command. For a takeoff command (22), it's the
                    minimum pitch.
                6. Param2: This is the second parameter and its meaning
                    depends on the command. For a change speed command (178),
                    it's the new speed.
                7. Param3: This is the third parameter and its meaning depends
                    on the command.
                8. Param4: This is the fourth parameter and its meaning
                    depends on the command. For a simple waypoint command
                    (16), it's the desired yaw angle.
                9. X/Param5: For a simple waypoint command (16),
                    this is the latitude.
                10. Y/Param6: For a simple waypoint command (16),
                    this is the longitude.
                11. Z/Param7: For a simple waypoint command (16),
                    this is the altitude.
                12. AutoContinue: If this is set to 1, the mission will
                    continue to the next waypoint automatically when the
                    command is finished.

             Methods:
                - `to_dict()`: returns a dict of the class parameters
            '''
            def __init__(self):
                self.current = 0
                self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT  # (16)
                self.param1 = 0
                self.param2 = 0
                self.param3 = 0
                self.param4 = 0
                self.param5 = 0
                self.param6 = 0
                self.param7 = 0
                self.auto = 1
                self.mission_type = 0

            def to_dict(self) -> dict:
                '''
                Returns a dict of this class' parameters

                Return:
                    `dict`: Dictionary of this class' parameters
                '''
                excluded_vars = {
                        # '_is_new',
                        # 'timestamp'
                    }
                return {
                    key: value for key, value in vars(self).items()
                    if key not in excluded_vars}

        class Waypoint(_MissionWaypoint):
            def __init__(self,
                         lat: float = 0,
                         lon: float = 0,
                         alt: float = None,
                         delay: int = 0,
                         ):
                '''
                (_MissionWaypoint) The Copter will fly a straight line
                to the specified latitude, longitude , and altitude.
                It will then wait at the point for a specified delay
                time and then proceed to the next waypoint.

                #### Params:
                    `lat (float)`: latitude. If 0, then use current latitude
                    `lon (float)`: longitude. If 0, then use current longitude
                    `alt (float)`: altitude. If 0, then use current altitde
                    `delay (int)`: Hold time in integer seconds - MAX 65535 sec

                #### Return:
                    None
                '''

                super().__init__()
                self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                self.param1 = delay
                self.param5 = int(lat*1e7)
                self.param6 = int(lon*1e7)
                self.param7 = alt

            @classmethod
            def create_waypoint(cls, lat: float = 0, lon: float = 0,
                                alt: float = None, delay: int = 0):

                return cls(lat, lon, alt, delay)

        class SplineWaypoint(_MissionWaypoint):
            def __init__(self,
                         lat: float = 0,
                         lon: float = 0,
                         alt: float = None,
                         delay: int = 0
                         ):
                '''
                (_MissionWaypoint) The Copter will fly a straight line to the
                specified latFly to the target location using a spline path,
                then wait (hover) for a specified time before proceeding
                to the next command.

                #### Params:
                    `lat (float)`: latitude. If 0, then use current latitude
                    `lon (float)`: longitude. If 0, then use current longitude
                    `alt (float)`: altitude. If 0, then use current altitde
                    `delay (int)`: Hold time in integer seconds - MAX 65535 sec

                #### Return:
                    None
                '''

                super().__init__()
                self.command = mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT
                self.param1 = delay
                self.param5 = int(lat*1e7)
                self.param6 = int(lon*1e7)
                self.param7 = alt

            @classmethod
            def create_waypoint(cls, lat: float = 0, lon: float = 0,
                                alt: float = None, delay: int = 0):

                return cls(lat, lon, alt, delay)

        class ChangeSpeed(_MissionWaypoint):
            def __init__(self,
                         speed: float = 5,
                         speed_type: int = 1,
                         throttle: int = -1
                         ):
                '''
                (_MisssionWaypoint) The Copter will change the speed for all
                follow=on maneuves

                Args:
                    `speed (float)`: speed in m/s
                    `speed_type (int)`: 0=airspeed, 1=groundspeed
                                        2=climb_speed, 3=descent_speed
                    `throttle (int)`: % of throttle application
                        (-1=no change, -2=return to default)
                '''

                super().__init__()
                self.command = mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
                self.param1 = speed_type
                self.param2 = speed
                self.param3 = throttle

            @classmethod
            def create_waypoint(cls, speed: float = 5,
                                speed_type: int = 1,
                                throttle: int = -1):

                return cls(speed, speed_type, throttle)

        class CircleTurns(_MissionWaypoint):
            def __init__(self,
                         lat: float = 0,
                         lon: float = 0,
                         alt: float = 0,
                         turns: Union[int, float] = 0,
                         radius: int = 0
                         ):
                '''
                (_MissionWaypoint)  Circles the specified location for a
                specified number of turns, and then proceed to the next
                command. If zero is specified for a latitude/longitude/altitude
                parameter then the current location value for the parameter
                will be used. Fractional turns between 0 and 1 are supported,
                while turns greater than 1 must be integers.

                #### Params:
                    `lat (float)`: latitude. If 0, then use current latitude
                    `lon (float)`: longitude. If 0, then use current longitude
                    `alt (float)`: altitude. If 0, then use current altitde
                    `turns (int, float)`: Number of turns before moving on.
                        Fractional turns between 0 - 1 are supported.
                        Turns greater than 1 must be integers (pos CW/neg CCW)
                    `radius (int)`: radius of circle in meters

                Return:
                    None
                '''

                super().__init__()
                self.command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
                self.param1 = turns
                self.param3 = radius
                self.param5 = int(lat*1e7)
                self.param6 = int(lon*1e7)
                self.param7 = alt

            @classmethod
            def create_waypoint(cls, lat: float, lon: float,
                                alt: float, turns: Union[int, float] = 0,
                                radius: int = 0):

                return cls(lat, lon, alt, turns, radius)

        class Takeoff(_MissionWaypoint):
            def __init__(self,
                         alt: float = 0
                         ):
                '''
                (_MissionWaypoint) Takeoff (either from the ground
                or byhand-launch). It should be the first command of
                nearly all Plane and Copter missions.

                Args:
                    `alt (float)`: altitude

                Return: None
                '''

                super().__init__()
                self.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                self.param7 = alt

            @classmethod
            def create_waypoint(cls, alt: float):

                return cls(alt)

        class Land(_MissionWaypoint):
            def __init__(self,
                         lat: float = 0,
                         lon: float = 0
                         ):
                '''
                (_MissionWaypoint) Takeoff (either from the ground
                or byhand-launch). It should be the first command of
                nearly all Plane and Copter missions.

                #### Params:
                    `lon (float)`: longitude. If 0, then use current longitude
                    `alt (float)`: altitude. If 0, then use current altitde

                #### Return:
                    None
                '''

                super().__init__()
                self.command = mavutil.mavlink.MAV_CMD_NAV_LAND
                self.param5 = lat
                self.param6 = lon

            @classmethod
            def create_waypoint(cls, lat: float, lon: float):

                return cls(lat, lon)

    # ### Default Command Methods ###

    def send_long_command(self,
                          command,
                          confirmation,
                          param1,
                          param2,
                          param3,
                          param4,
                          param5,
                          param6,
                          param7,
                          verify=False) -> bool:

        if self._mav_connection is not None:
            result_event = threading.Event()
            result_container = [None]  # A mutable container to store result

            thread = threading.Thread(
                target=self._send_long_command, args=(
                    command,
                    confirmation,
                    param1,
                    param2,
                    param3,
                    param4,
                    param5,
                    param6,
                    param7,
                    result_event,
                    result_container,
                    verify
                ),
                daemon=True
            )
            thread.start()
            result_event.wait()
            return result_container[0]  # Return the result stored in container

    def _send_long_command(self,
                           command,
                           confirmation,
                           param1,
                           param2,
                           param3,
                           param4,
                           param5,
                           param6,
                           param7,
                           result_event,
                           result_container,
                           verify=False):

        with self._lock:
            if self._retry_count >= 3:
                result_container[0] = False
                result_event.set()
                return

            self._mav_connection.mav.command_long_send(
                self._mav_connection.target_system,
                self._mav_connection.target_component,
                command,
                1,  # confirmation
                param1, param2, param3, param4, param5, param6, param7
            )

        if verify:
            start_time = time.time()
            ack_received = False

            while (time.time() - start_time < 3) and not ack_received:
                msg_dict = (self._agent_hub.
                            mavlink_manager.current_mavlink_message_dict)
                if "COMMAND_ACK" in msg_dict:
                    msg = msg_dict["COMMAND_ACK"]
                    # print(msg)
                    if msg.command == command and msg.result == 0:
                        ack_received = True
                        with self._lock:
                            self._retry_count = 0
                        del (self._agent_hub.mavlink_manager.
                             current_mavlink_message_dict["COMMAND_ACK"])

            if ack_received:
                with self._lock:
                    self._retry_count = 0
                result_container[0] = True
                result_event.set()
            else:
                with self._lock:
                    self._retry_count += 1
                self._send_long_command(
                    command,
                    confirmation,
                    param1,
                    param2,
                    param3,
                    param4,
                    param5,
                    param6,
                    param7,
                    result_event,
                    result_container,
                    verify
                )
        else:
            result_container[0] = True
            result_event.set()

    def send_int_command(self,
                         frame,
                         command,
                         current,
                         autocontinue,
                         param1,
                         param2,
                         param3,
                         param4,
                         x,
                         y,
                         z,
                         verify=False) -> bool:

        if self._mav_connection is not None:
            result_event = threading.Event()
            result_container = [None]  # A mutable container to store result

            thread = threading.Thread(
                target=self._send_int_command, args=(
                    frame,
                    command,
                    current,
                    autocontinue,
                    param1,
                    param2,
                    param3,
                    param4,
                    x,
                    y,
                    z,
                    result_event,
                    result_container,
                    verify
                ),
                daemon=True
            )
            thread.start()
            result_event.wait()
            return result_container[0]  # Return the result stored in container

    def _send_int_command(self,
                          frame,
                          command,
                          current,
                          autocontinue,
                          param1,
                          param2,
                          param3,
                          param4,
                          x,
                          y,
                          z,
                          result_event,
                          result_container,
                          verify=False
                          ):

        with self._lock:
            if self._retry_count >= 3:
                result_container[0] = False
                result_event.set()
                return

            self._mav_connection.mav.command_int_send(
                self._mav_connection.target_system,
                self._mav_connection.target_component,
                frame,
                command,
                current,
                autocontinue,
                param1,
                param2,
                param3,
                param4,
                x,
                y,
                z
            )

        if verify:
            start_time = time.time()
            ack_received = False

            while (time.time() - start_time < 3) and not ack_received:
                msg_dict = (self._agent_hub.
                            mavlink_manager.current_mavlink_message_dict)
                if "COMMAND_ACK" in msg_dict:
                    msg = msg_dict["COMMAND_ACK"]
                    if msg.command == command and msg.result == 0:
                        ack_received = True
                        with self._lock:
                            self._retry_count = 0
                        del (self._agent_hub.mavlink_manager.
                             current_mavlink_message_dict["COMMAND_ACK"])

            if ack_received:
                with self._lock:
                    self._retry_count = 0
                result_container[0] = True
                result_event.set()
            else:
                with self._lock:
                    self._retry_count += 1
                self._send_long_command(
                    frame,
                    command,
                    current,
                    autocontinue,
                    param1,
                    param2,
                    param3,
                    param4,
                    x,
                    y,
                    z,
                    result_event,
                    result_container,
                    verify
                )
        else:
            result_container[0] = True
            result_event.set()

    def send_int_mission_items(self,
                               mission_waypoints: List[wpt._MissionWaypoint],
                               verify: bool = False) -> bool:
        '''

        Upload a list of _MissionWaypoints (ex: .wpt.Waypoint()) as a list.

        #### Params:
            `mission_waypoints ([_MissionWaypoints])`: list of mission
                waypoints called for example: by something like 
                agent_command_manager.wpt.Waypoint()

        #### Return:
            bool
        '''

        if self._mav_connection is not None:
            result_event = threading.Event()
            result_container = [None]  # A mutable container to store result

            thread = threading.Thread(
                target=self._send_int_mission_items, args=(
                    mission_waypoints,
                    result_event,
                    result_container,
                    verify
                ),
                daemon=True
            )
            thread.start()
            result_event.wait()
            # thread.join()
            return result_container[0]  # Return the result stored in container

    def _send_int_mission_items(self,
                                mission_waypoints,
                                result_event,
                                result_container,
                                verify: bool = False
                                ) -> bool:

        with self._lock:
            if self._retry_count >= 3:
                result_container[0] = False
                result_event.set()
                return

        self._mav_connection.mav.mission_clear_all_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component
        )

        n = len(mission_waypoints)
        # This send clears the previous mission and loads preps
        # the ArduPilot for n new waypoints
        self._mav_connection.mav.mission_count_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            n+1,
            0
        )

        # Tells the ArduPilot to get ready for new points
        self._ack("MISSION_REQUEST")

        first_item = copy.deepcopy(mission_waypoints[0])
        first_item.seq = 0
        mission_waypoints.insert(0, first_item)

        for i, waypoint in enumerate(mission_waypoints):
            self._mav_connection.mav.mission_item_int_send(
                self._mav_connection.target_system,
                self._mav_connection.target_component,
                i,
                waypoint.frame,
                waypoint.command,
                waypoint.current,
                waypoint.auto,
                waypoint.param1,
                waypoint.param2,
                waypoint.param3,
                waypoint.param4,
                waypoint.param5,
                waypoint.param6,
                waypoint.param7,
                waypoint.mission_type
            )

        start_time = time.time()
        msn_ack_received = False

        while (time.time() - start_time < 22) and not msn_ack_received:
            # print("Try1")
            ack_msg = self._mav_connection.recv_match(
                type='MISSION_ACK', blocking=True, timeout=3)
            if ack_msg is not None:
                ack_msg = ack_msg.to_dict()
                msn_ack_received = True
                with self._lock:
                    self._retry_count = 0
                ack_result = ack_msg['type']\
                    == mavutil.mavlink.MAV_MISSION_ACCEPTED
                if verify:
                    result_container[0] = ack_result
                    result_event.set()
                    return
                else:
                    result_container[0] = True
                    result_event.set()
                    return
            else:
                with self._lock:
                    self._retry_count += 1

                self._mav_connection.mav.mission_clear_all_send(
                    self._mav_connection.target_system,
                    self._mav_connection.target_component
                )

                n = len(mission_waypoints)
                # This send clears the previous mission and loads preps
                # the ArduPilot for n new waypoints
                self._mav_connection.mav.mission_count_send(
                    self._mav_connection.target_system,
                    self._mav_connection.target_component,
                    n+1,
                    0
                )

                # Tells the ArduPilot to get ready for new points
                self._ack("MISSION_REQUEST")

                first_item = copy.deepcopy(mission_waypoints[0])
                first_item.seq = 0
                mission_waypoints.insert(0, first_item)

                for i, waypoint in enumerate(mission_waypoints):
                    self._mav_connection.mav.mission_item_int_send(
                        self._mav_connection.target_system,
                        self._mav_connection.target_component,
                        i,
                        waypoint.frame,
                        waypoint.command,
                        waypoint.current,
                        waypoint.auto,
                        waypoint.param1,
                        waypoint.param2,
                        waypoint.param3,
                        waypoint.param4,
                        waypoint.param5,
                        waypoint.param6,
                        waypoint.param7,
                        waypoint.mission_type
                    )

        print("Load Failed")
        result_container[0] = False
        result_event.set()

    def set_position_target(self,
                            lat: float = None,
                            lon: float = None,
                            relative_alt: float = None,
                            msl_alt: float = None,
                            velxyz:
                            Optional[Tuple[float, float, float]] = None,
                            accelxyz:
                            Optional[Tuple[float, float, float]] = None,
                            hdg=None,
                            yawRate=None,
                            ):
        if self._mav_connection is not None:

            thread = threading.Thread(
                target=self._set_position_target, args=(
                    lat,
                    lon,
                    relative_alt,
                    msl_alt,
                    velxyz,
                    accelxyz,
                    hdg,
                    yawRate,
                ),
                daemon=True
            )
            thread.start()

    def _set_position_target(self,
                             lat,
                             lon,
                             relative_alt,
                             msl_alt,
                             velxyz,
                             accelxyz,  # unimplemented
                             yaw,
                             yawRate,
                             ):
        '''
        Class method that adds detail and filtering to the
        chosen cmd_nav_goto... command
        '''

        with self._lock:

            _lat = 0
            _lon = 0
            _alt = 0
            _velX = 0
            _velY = 0
            _velZ = 0
            _accX = 0
            _accY = 0
            _accZ = 0
            _yaw = 0
            _yawRate = 0.5

            # a list of all the masks used in the command
            mask_list = []
            # for the mask to change params, use:
            #   bitmap to show what to IGNORE (1 means ignore, 0 means use)
            #   0b[yaw rate][yaw][force set][az][ay][ax][vz][vy][vx][z][y][x]
            #   Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
            #   Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
            #   Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
            #   Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
            #   Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
            #   Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
            #   Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)

            # Set the standard reference frame
            # ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT
            # ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

            if (relative_alt is not None) and (msl_alt is not None):

                raise ValueError("relative_alt and msl_alt cannot \
                                be set at the same time")

            else:

                if lat is not None:
                    _lat = int(float(lat) * 1e7)
                    mask_list.append(0b111111111110)

                if lon is not None:
                    _lon = int(float(lon) * 1e7)
                    mask_list.append(0b111111111101)

                if lat is not None or lon is not None:
                    msg_type = "MAVLink_set_position_target_global_int_message"
                    ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT

                if relative_alt is not None:
                    _alt = relative_alt
                    mask_list.append(0b111111111011)
                    ref_frame = (mavutil.mavlink.
                                 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)

                if msl_alt is not None:
                    _alt = msl_alt
                    mask_list.append(0b111111111011)
                    ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT

                if velxyz is not None:
                    print(f"velxyz: {velxyz}")
                    _velX = velxyz[0]
                    _velY = velxyz[1]
                    _velZ = velxyz[2]
                    mask_list.append(0b111111000111)
                    msg_type = "MAVLink_set_position_target_local_ned_message"
                    ref_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

                # Update yaw value and mask_list if yaw is used to rad
                if yaw is not None:
                    _yaw = math.radians(yaw)
                    mask_list.append(0b101111111111)
                else:
                    _yaw = 0

                # Update yawRate value and mask_list if yawRate is used to rad
                # print(f"yaw rate: {yawRate}")
                if yawRate is not None:
                    _yawRate = math.radians(yawRate)
                    mask_list.append(0b011111111111)
                else:
                    _yawRate = 0

                final_mask = 0b110111111111
                # print(f"mask-i: {final_mask:012b}")
                for mask in mask_list:
                    # print(f"mast-n: {mask:012b}")
                    final_mask = final_mask & mask
                    # print(f"mast-u: {final_mask:012b}")

                # print(ref_frame)
                # print(f"mask-f: {final_mask:012b}")

                message_function = getattr(mavutil.mavlink, msg_type)

                self._mav_connection.mav.send(
                    message_function(
                        0,
                        self._mav_connection.target_system,
                        self._mav_connection.target_component,
                        ref_frame,        # Frame of Reference
                        int(final_mask),  # mask of desired params
                        _lat,     # latitude * 1e7
                        _lon,     # lon * 1e7
                        _alt,     # alt in meters MSL
                        _velX,    # X velocity in m/s (positive is North)
                        _velY,    # Y velocity in m/s (positive is East)
                        _velZ,    # Z velocity in m/s (positive is down)
                        _accX,    # X acceleration in m/s/s (pos is North)
                        _accY,    # Y acceleration in m/s/s (pos is East)
                        _accZ,    # Z acceleration in m/s/s (pos is Down)
                        _yaw,     # yaw or heading in radians (0 is forward)
                        _yawRate  # yaw rate in rad/s

                    )
                )

    def _vprint(self, print_string):
        '''
        This class is used in conjunction with the verbose argument to
        determine whether to show the expanded data in print_string

        Args:
            `print_string (str)`: The string to print

        Return: None
        '''
        if self._verbose:
            print(print_string)
