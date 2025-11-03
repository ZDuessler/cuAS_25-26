from datetime import datetime
from abc import ABC
import math

import os
from pymavlink import mavutil
os.environ['MAVLINK20'] = '1'

MAV_STATE_DICT = {
    0: "UNINIT",
    1: "BOOT",
    2: "CALIBRATING",
    3: "STANDBY",
    4: "ACTIVE",
    5: "CRITICAL",
    6: "EMERGENCY",
    7: "POWEROFF",
    8: "TERMINATION"
}

ARM_STATE_DICT = {
    0: "prearm-fail",
    1: "MAVLINK Lost",
    2: "prearm-good",
    3: "armed-idle",
    4: "armed-above idle"
}


class DynamicStatusClass(ABC):

    def __init__(self, **kwargs):
        self._is_new = True  # Flag to track if the data is new
        self.timestamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        self.update_attributes(kwargs)
        self.message_class = kwargs.get('_message_class', None)

    def update_attributes(self, new_data):

        # Check to see if there are changes in the data
        # (with the timestamp there should always be changes)
        obj_data_dict = self.to_dict().copy()
        new_data_dict = new_data.copy()
        if obj_data_dict != new_data_dict:
            for key, value in new_data.items():
                setattr(self, key, value)
            setattr(self, 'timestamp',
                    datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f"))

        # Check to see if there are changes in any of the data other than
        # the timestamp.  This will trigger the is_new parameter
        if 'timestamp' in obj_data_dict:
            del obj_data_dict['timestamp']
        if 'timestamp' in new_data_dict:
            del new_data_dict['timestamp']
        if obj_data_dict != new_data_dict:
            self._is_new = True

    def to_dict(self):
        excluded_vars = {
                '_is_new',
                '_message_class',
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


class AgentPosition(DynamicStatusClass):
    '''
    The current data from GLOBAL_POSITION_INT converted to decimal

    Args:
        `lat (float)`: latitude
        `lon (float)`: longitude
        `alt (float)`: MSL altitude (m)
        `relative_alt (float)`: height above takeoff (HAT) (m)
        `vx (float)`: vground X speed (cm/s) (Latitude, positive north)
        `vy (float)`: vground Y speed (cm/s) (Longitude, positive east)
        `vz (float)`: vground Z speed (cm/s) (Altitude, positive down)
        `hdg (float)`: vehicle heading (yaw angle) (deg)
        `hdg_rad (float)`: vehicle heading (yaw angle) (rad)
    '''

    def __init__(self, lat=0.0, lon=0.0, alt=0.0, relative_alt=0.0,
                 vx=0.0, vy=0.0, vz=0.0, hdg=0.0, hdg_rad=0.0):
        super().__init__(lat=lat/1e7, lon=lon/1e7,
                         alt=alt/1e3, relative_alt=relative_alt/1e3,
                         vx=vx, vy=vy, vz=vz,
                         hdg=hdg/1e2, hdg_rad=hdg/1e2*math.pi/180)
        self.message_class = 'agent_position'

    def __call__(self):
        return self.to_dict()

    def __getattr__(self, name):
        if name in self.__dict__:
            return self.__dict__[name]
        raise AttributeError(f"'{self.__class__.__name__}' "
                             f"object has no attribute '{name}'")

    def __getitem__(self, key):
        return self.to_dict()[key]

    def __setitem__(self, key, value):
        self.to_dict()[key] = value

    def __str__(self):
        return str(self.to_dict())


class AgentStatus:
    '''
    This retains important current states for the agent. There are default
    states shown below but it also retains the current state for all requested
    MAVLink messages.

    The AgentStatus object also implements a 'has_new' method which returns
    whether or not any of the AgentStatus parameters have been updated since
    the last time the user requested that particular parameter.

    Attr:
        `timestamp (str)`: timestamp of the last time anyting in status changed
        `agent_id (str)`: unique id given to this agent via YAML file
        `agent_poisition (AgentPosition)`: object with current nav data
        `flight_mode (dict{"mode": None, "timestamp": datetime})`: \
            current flight mode (RTL, GUIDED, AUTO, etc.)
        `sys_status (dict{"mav_state": None, "timestamp": datetime})`: \
            This is the current mavlink state of the ArduPilot
            https://mavlink.io/en/messages/common.html#MAV_STATE
        `prearm_status (dict{"status": None, "timestamp": datetime})`: \
            Checks if all the pre-arm checks successfully passed.
            Note: This can still be True regardless of flight mode (like RTL)
        `arm_state (dict{"state": None, "timestamp": datetime)`: \
            Current arm-state of the ArduPilot.
            [0: "prearm-fail", 1: "MAVLINK Lost", 2: "prearm-good",
            3: "armed-idle", 4: "armed-above idle"]
        `battery_health (dict{"state": 0, "timestamp": datetime})`: \
            Current battery remaining in %

    Methods:
        - `has_new (key: str)`: You can call this at any time to see if the
         parameter defined by key has a new value
        - `build_message_object (msg_type: str, input_dict: dict)`: Add a new
         parameter to the AgentStatus object.
    '''

    def __init__(self, config: dict):

        # The timestamp of the last time this AgentStatus object was changed
        self.timestamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        self._config = config

        # The agent ID set in the agent_configuration YAML file
        self.agent_id = self._config["AGENT_ID"]

        # The agent's current position
        self.agent_position = AgentPosition()

        # Set some higher fidelity parameters in the class based on standard
        # input messages
        self.build_message_class("flight_mode", {"mode": None})
        self.build_message_class("sys_status", {"mav_state": None})
        self.build_message_class("prearm_status", {"status": None})
        self.build_message_class("arm_state", {"state": None})
        self.build_message_class("battery_health", {"state": 0})

    def has_new(self, key):
        # This function looks to see if the parameter you are asking for
        # has a new value.  The 'key' is a string of the parameter name
        # This can be MAVLink message names such as "HEARTBEAT" or
        # custom messages such as "flight_mode"
        if hasattr(self, key):
            try:
                return getattr(self, key).is_new
            except NameError:
                pass
        return False

    def build_message_class(self, msg_type: str, input_dict: dict):
        '''
        This class builds out the AgentStatus' desired new parameter
        If it's a straight MAVLink message, it will name the parameter
        the message name (such as HEARTBEAT) and will be defined in all caps
        If it's a custom parameter, it will be as it's defined in msg_type
        ex: self.build_message_class("flight_mode", {"mode": None})
        '''
        self.timestamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        original_msg_dict = input_dict
        if 'mavpackettype' in original_msg_dict.keys():
            del original_msg_dict['mavpackettype']
        msg_dict = {msg_type: original_msg_dict}
        # Add a timestamp to the dictionary data
        for key, value in msg_dict.items():
            value['_message_class'] = key
            dynamic_class = type(
                    key,
                    (DynamicStatusClass,),
                    {}
                )
            # Instantiate an object of the dynamically created class
            instance = dynamic_class(**value)
            setattr(self, key, instance)

    def update_message_object(self, msg_from_ardupilot):
        # Update the timestamp for the AgentStatus object anytime a parameter
        # gets updated
        self.timestamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")

        # This updates the already generated parameter objects of the
        # AgentStatus object with new data.  This will also trigger the
        # is_new attribute of the object so that the has_new() method
        # will return True
        msg_type = msg_from_ardupilot.get_type()
        msg_dict = msg_from_ardupilot.to_dict()
        del msg_dict['mavpackettype']
        obj = getattr(self, msg_type)
        obj.update_attributes(msg_dict)

        # ### Higher fidelity class parameters from the default messages ###
        if msg_type == 'GLOBAL_POSITION_INT':
            # Update the AgentPosition object within the AgentStatus
            msg_dict["lat"] = msg_dict['lat']/1e7
            msg_dict["lon"] = msg_dict['lon']/1e7
            msg_dict["alt"] = msg_dict['alt']/1e3
            msg_dict["relative_alt"] = msg_dict['relative_alt']/1e3
            msg_dict["hdg"] = msg_dict['hdg']/1e2
            msg_dict["hdg_rad"] = msg_dict['hdg']*math.pi/180
            self._update_custom_class_object(self.agent_position, msg_dict)

        if msg_type == 'HEARTBEAT':
            # Update the flight_mode
            mode = mavutil.mode_string_v10(msg_from_ardupilot)
            self._update_custom_class_object(
                self.flight_mode, {"mode": mode})
            # Update the mav_state.  This is general ArduPilot
            # readiness such as booting up, calibration, etc.
            # It will not trigger for things like out of the Fence.
            mav_state = MAV_STATE_DICT[msg_dict['system_status']]
            self._update_custom_class_object(
                self.sys_status, {"mav_state": mav_state})

        if msg_type == 'SYS_STATUS':
            # Update the battery remaining
            self._update_custom_class_object(
                self.battery_health, {"state": msg_dict['battery_remaining']}
            )

            # Get the prearm status - Whether (T/F) you can arm the drone
            bitmask_health = int(
                msg_from_ardupilot.to_dict()['onboard_control_sensors_health'])
            status_bits = format(bitmask_health, "032b")
            status_to_check = [mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK]
            prearm_status = self._check_MAV_SYS_STATUS_SENSOR_prearm_status(
                status_bits, status_to_check)[268435456]

            self._update_custom_class_object(
                self.prearm_status, {"status": prearm_status})

            # Get the current arm state of the drone
            if (prearm_status) is False:
                final_prearm_status = 0
            else:
                if (self.HEARTBEAT.base_mode == 89 or
                        self.HEARTBEAT.base_mode == 81):
                    final_prearm_status = 2
                elif (self.HEARTBEAT.base_mode == 209 or
                        self.HEARTBEAT.base_mode == 217):
                    final_prearm_status = 3
            if self.HEARTBEAT.system_status == 4:
                final_prearm_status = 4

            self._update_custom_class_object(
                self.arm_state, {"state": ARM_STATE_DICT[final_prearm_status]})

    def _update_custom_class_object(self, obj, msg_input_dict):
        # This is a helper function that iterates through the keys of the
        # passed dictionary (for example the MAVLink message data) and
        # populates the associated AgentStatus parameter that represents
        # that MAVLink message
        obj_vars = list(obj.to_dict().keys())
        # print(obj_vars)

        # Ensure 'name' is present in msg_input_dict
        msg_input_dict['message_class'] = getattr(obj, 'message_class', None)

        msg_input_dict['timestamp'] = \
            datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        # print(msg_input_dict)
        obj_var_dict = {}
        for obj_var in obj_vars:
            obj_var_dict[obj_var] = msg_input_dict.get(obj_var, None)
        obj.update_attributes(obj_var_dict)

    def _check_MAV_SYS_STATUS_SENSOR_prearm_status(
            self, status_bits, check_status_ids):
        # Helper function to determine whether the agent is ready to arm or not
        running_status = dict()
        for status_id in check_status_ids:
            pos = int(math.log(status_id, 2)) + 1
            if str(status_bits)[32-int(pos)] == '1':
                running_status[status_id] = True
            else:
                running_status[status_id] = False

        return running_status
