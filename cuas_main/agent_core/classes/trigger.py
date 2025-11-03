# import sys
# import math
import ast
import re
import threading
from datetime import datetime
import math

# from collections import defaultdict
# from typing import TYPE_CHECKING

# if TYPE_CHECKING:
#     from c3_node import C3Node

'''
__init__(self, ...): The constructor method, called when an object is created.
__str__(self): Defines the string representation of an object (used by str() and print()).
__repr__(self): Defines the unambiguous representation of an object (used by repr()).
__len__(self): Returns the length of an object (used by len()).
__getitem__(self, key): Allows indexing and slicing (used by square brackets []).
__setitem__(self, key, value): Allows setting values using indexing.
__call__(self, ...): Allows an object to be called like a function.
__add__(self, other): Defines behavior for addition (+ operator).
__sub__, __mul__, __div__, etc.: Similar methods for other arithmetic operations.
__getattr__(self, name): Called when an attribute is not found.
__setattr__(self, name, value): Called when an attribute is set.
__delattr__(self, name): Called when an attribute is deleted.
'''  # noqa: E501


class TriggerManager:

    def __init__(self, c3Node):
        self.triggers = {}
        self.vars = {}
        self._connected_agents = c3Node.connected_clients

    # ########################################################

    class Trigger:

        def __init__(self,
                     trigger_id,
                     c3nodeVar,
                     threshold: any,
                     operator: str,
                     repeat: bool = False,
                     agents: str = "",
                     key: str = None,
                     count=None):

            self.trigger_id = trigger_id
            self.c3NodeVar = c3nodeVar
            self.threshold = threshold
            self.operator = operator
            self.agents = agents
            self.repeat = repeat
            self.key = key
            self.count = count

            self.timestamp = datetime.now()

            self.triggered = False
            # self.cleared_condition = {}

            self.triggered_agents = []

            # collects the value of each agent's conditional result
            # as it cycles through the for loop
            # so the any, all, subset ouput can be evaluated
            self.persistant_status = {}

            # When using a transition conditional (l->h, h->l, ->=, =->)
            # this checks if the value has already been read
            self.transition_value = None

            # Parse count if it includes a comparison operator
            self.comparison_operator = None
            self.comparison_value = None
            if isinstance(count, str):
                match = re.match(r"([<>]=?|==)\s*(\d+%?)", count)
                if match:
                    self.comparison_operator = match.group(1)
                    self.comparison_value = match.group(2)
                    # print(f"op: {self.comparison_operator}")
                    # print(f"val: {self.comparison_value}")

        def compare_date(self, new_date) -> str:
            if new_date > self.timestamp:
                # print(f"New date: {self.timestamp} -> {new_date}")
                self.timestamp = new_date
                return new_date

    class MsgTrigger:

        def __init__(self,
                     manager,
                     c3nodeVarName: str,
                     threshold: any,
                     operator: str,
                     repeat: bool = False,
                     agents: str = "",
                     key: str = None,
                     count=None):

            self.manager = manager
            self.response = False
            self.triggered_agents = []

            self._create_msg_trigger(c3nodeVarName,
                                     threshold,
                                     operator,
                                     repeat,
                                     agents,
                                     key,
                                     count)

        def __get_item__(self, index):
            if index == 0:
                return self.response
            elif index == 1:
                return self.triggered_agents
            else:
                raise IndexError("Index out of range")

        def __bool__(self):
            return self.response

        def _create_msg_trigger(
                    self,
                    c3nodeVarName: str,
                    threshold: any,
                    operator: str,
                    repeat: bool = False,
                    agents: str = "",
                    key: str = None,
                    count=None):

            # If the c3NodeVar that the trigger is esablished around
            # is not evaluating yet then don't run this trigger
            if self.manager.vars[c3nodeVarName] is None:
                return

            trigger_id = f"{c3nodeVarName}_{threshold}_"\
                         f"{operator}_{repeat}_{agents}_{key}_{count}"

            # If this is the first time calling the trigger, then add
            # it to the dictionary of triggers
            if trigger_id not in self.manager.triggers.keys():
                trgr = TriggerManager.Trigger(trigger_id,
                                              self.manager.vars[c3nodeVarName],
                                              threshold,
                                              operator,
                                              repeat,
                                              agents,
                                              key,
                                              count)
                self.manager.triggers[trigger_id] = trgr

            # establish the trgr variable for this trigger
            trgr: TriggerManager.Trigger = self.manager.triggers[trigger_id]

            # If the trigger has already fired and it's not a repeat trigger
            # Then don't run the trigger code
            if (trgr.triggered) and (trgr.repeat is False):
                self.response = False
                return

            # If the trigger has already fired and it's a repeat trigger,
            # continue to return False until all agents' date is
            # refreshed. This is checked by looking at each agent's timestamps
            if trgr.triggered and trgr.repeat:

                keys_to_delete = []
                for agent in trgr.persistant_status.keys():
                    try:
                        if ((self.manager.vars[c3nodeVarName].
                                msg_dict['agents'][agent]['timestamp'] !=
                                trgr.persistant_status[agent]['timestamp']) and
                                (trgr.persistant_status[agent]['timestamp']
                                 is not None)):
                            keys_to_delete.append(agent)
                    except TypeError:
                        pass

                for agent in keys_to_delete:

                    # print(f"Here D: {agent}")
                    del trgr.persistant_status[agent]
                    if agents == 'any' or agents == 'first':
                        trgr.triggered = False
                        self.response = False
                        return

                if trgr.persistant_status != {}:
                    self.response = False
                    return
                else:
                    trgr.triggered = False

            # Check if the msg_dict has been updated by external sources using the
            # trigger's timestamp variable (this is not the agent's timestamp)
            if trgr.c3NodeVar.msg_dict['timestamp'] is not None:
                timestamp = trgr.c3NodeVar.msg_dict['timestamp']
                if timestamp > trgr.timestamp:
                    trgr.timestamp = timestamp
                else:
                    self.response = False
                    return
            else:
                self.response = False
                return

            # print(f"Curr: {self.manager.vars[c3nodeVarName].msg_dict}")
            # print(f"Prev: {self.manager.vars[c3nodeVarName].previous}")

            # Get the agent keys based on the 'agents' parameter
            if trgr.agents == 'all':
                agent_keys = list(self.manager._connected_agents.keys())
            elif trgr.agents == 'any':
                agent_keys = list(self.manager.vars[c3nodeVarName].
                                  msg_dict['agents'].keys())
            elif trgr.agents[0] == '[':
                agent_keys = [str(item) for item in ast.
                              literal_eval(trgr.agents)]

            # You can only look at agent_id's if they exist in the
            # self.manager.vars[c3nodeVarName].msg_dict['agents'].keys()
            # However, the matching count is compared to the total number
            # of agent_ids given in 'agents'
            valid_agent_keys = [agent for agent in agent_keys if
                                agent in
                                list(self.manager.vars[c3nodeVarName].
                                     msg_dict['agents'].keys())]
            matching_agents = []

            # Process Transition Parameters
            operators = {
                            '>': [
                                  lambda x, z: x > z,
                                  ],
                            '>=': [
                                  lambda x, z: x >= z,
                                  ],
                            '<': [
                                  lambda x, z: x < z,
                                  ],
                            '<=': [
                                  lambda x, z: x <= z,
                                  ],
                            '==': [
                                  lambda x, z: x == z,
                                  ],
                            '!=': [
                                  lambda x, z: x != z,
                                  ],
                            '->=': [
                                    lambda x, z: x == z,
                                    lambda y, z: y != z,
                                    lambda x, z: x != z,
                                    ],
                            '=->': [
                                    lambda x, z: x != z,
                                    lambda y, z: y == z,
                                    lambda x, z: x == z,
                                    ],
                            'h->l': [
                                     lambda x, z: x <= z,
                                     lambda y, z: y > z,
                                     lambda x, z: x > z,
                                     ],
                            'l->h': [
                                     lambda x, z: x >= z,
                                     lambda y, z: y < z,
                                     lambda x, z: x < z,
                                     ]}

            def evaluate_conditions(op, a, x, y, z, p):

                if len(operators[op]) == 3:

                    op1 = operators[op][0]
                    op2 = operators[op][1]
                    op3 = operators[op][2]

                    if op1(x[key], z) and op2(y, z):
                        return {'triggered': True,
                                'timestamp': x['timestamp']}
                    elif op3(x[key], z):
                        try:
                            return {'triggered': False,
                                    'timestamp': p['timestamp']}
                        except (KeyError):
                            pass
                    else:
                        return p[a]

                elif len(operators[op]) == 1:
                    op1 = operators[op][0]
                    if op1(x[key], z):
                        return {'triggered': True,
                                'timestamp': x['timestamp']}
                    else:
                        return p[a]

            try:
                # print(f"Persistant Before: {trgr.persistant_status}")
                for agent in valid_agent_keys:

                    # print(f"{agent} // C: {self.manager.vars[c3nodeVarName].msg_dict['agents'][agent][key]} "
                    #       f"P: {self.manager.vars[c3nodeVarName].previous[agent][0][key]}")

                    # Ensure `trgr.persistant_status` has a default entry for each agent
                    if agent not in trgr.persistant_status:
                        trgr.persistant_status[agent] = {'triggered': False,
                                                         'timestamp': None}

                    trgr.persistant_status[agent] = evaluate_conditions(
                        operator,
                        agent,
                        self.manager.vars[c3nodeVarName].msg_dict['agents'][agent],
                        self.manager.vars[c3nodeVarName].previous[agent][0][key],
                        threshold,
                        trgr.persistant_status
                    )

                for key, value in trgr.persistant_status.items():
                    if value and value.get('triggered'):
                        matching_agents.append(key)

                # print(f"Persistant After: {trgr.persistant_status}")
                # print(f"Matching Agents: {matching_agents}")

            except (IndexError):
                pass

            matching_count = len(matching_agents)
            total_agents = len(agent_keys)

            # If we are defining a percentage of agents that have to meet
            # the conditional criteria
            if trgr.comparison_value and '%' in trgr.comparison_value:
                comparison_value = int(
                    trgr.comparison_value.strip('%')) / 100 * total_agents
            elif count is None:
                comparison_value = total_agents
            # else we are defining a specific number of agents that
            # have to meet the conditional criteria
            else:
                comparison_value = int(trgr.comparison_value) if\
                    trgr.comparison_value else 0

            # Assuming the conditional response is not met - False
            response = False

            # if any of the agents that are sending data to the C3Node meet
            # the conditional requirement (i.e. matching count > 0) then return
            # a value of True

            if trgr.agents == 'any':
                response = matching_count > 0

            # else if you are looking at all the agents connected to the C3Node
            # or all the agents defined by the list given in 'agents'
            else:
                if trgr.comparison_operator:
                    if trgr.comparison_operator == '>=':
                        response = matching_count >= comparison_value
                    elif trgr.comparison_operator == '<=':
                        response = matching_count <= comparison_value
                    elif trgr.comparison_operator == '>':
                        response = matching_count > comparison_value
                    elif trgr.comparison_operator == '<':
                        response = matching_count < comparison_value
                    elif trgr.comparison_operator == '==':
                        response = matching_count == comparison_value
                else:
                    if total_agents != 0:
                        response = matching_count == total_agents

            # Set the trigger's matched agents for external assessment
            if response:
                self.triggered_agents = matching_agents

            trgr.triggered = response
            self.response = response
            return

    class ScalarTrigger:

        def __init__(self,
                     manager,
                     c3nodeVarName: str,
                     operator: str,
                     threshold: any,
                     repeat: bool = False
                     ):

            self.manager = manager
            self.response = False

            self._create_scalar_trigger(
                            c3nodeVarName,
                            operator,
                            threshold,
                            repeat)

        def __bool__(self):
            return self.response

        def __str__(self):
            return str(self.response)

        def _create_scalar_trigger(
                        self,
                        c3nodeVarName: str,
                        operator: str,
                        threshold: any,
                        repeat: bool = False
                        ):

            # If the c3NodeVar that the trigger is esablished around
            # is not evaluating yet then don't run this trigger
            if self.manager.vars[c3nodeVarName] is None:
                return

            trigger_id = f"{c3nodeVarName}_{threshold}_"\
                         f"{operator}_{repeat}"

            # If this is the first time calling the trigger, then add
            # it to the dictionary of triggers
            if trigger_id not in self.manager.triggers.keys():
                trgr = TriggerManager.Trigger(
                                        trigger_id,
                                        self.manager.vars[c3nodeVarName],
                                        threshold,
                                        operator,
                                        repeat)
                self.manager.triggers[trigger_id] = trgr
                # print(f"trigger_id: {trigger_id}")

            # evaluate the trigger
            trgr: TriggerManager.Trigger = self.manager.triggers[trigger_id]

            # If the trigger has already fired and it's not a repeat trigger
            if (trgr.triggered) and (trgr.repeat is False):
                self.response = False
                return

            # Process Transition Parameters
            operators = {
                         '>': [
                                lambda x, z: x > z
                              ],
                         '>=': [
                                lambda x, z: x >= z
                              ],
                         '<': [
                                lambda x, z: x < z
                              ],
                         '<=': [
                                lambda x, z: x <= z
                              ],
                         '==': [
                                lambda x, z: x == z
                              ],
                         '!=': [
                                lambda x, z: x != z
                              ],
                         '->=': [
                                 lambda x, z: x == z,
                                 lambda y, z: y != z,
                                 ],
                         '=->': [
                                 lambda x, z: x != z,
                                 lambda y, z: y == z,
                                 ],
                         'h->l': [
                                  lambda x, z: x <= z,
                                  lambda y, z: y > z,
                                  ],
                         'l->h': [
                                  lambda x, z: x >= z,
                                  lambda y, z: y < z,
                                  ]}

            def evaluate_conditions(op, x, y, z, trgr):

                if len(operators[op]) == 2:
                    op1 = operators[op][0]
                    op2 = operators[op][1]
                    try:
                        if op1(x, z) and op2(y, z):
                            if not trgr.repeat:
                                trgr.triggered = True
                            return True
                    except (KeyError):
                        pass
                else:
                    op1 = operators[op][0]
                    try:
                        if op1(x, z):
                            if not trgr.repeat:
                                trgr.triggered = True
                            return True
                    except (KeyError):
                        pass

            try:
                response = evaluate_conditions(
                    operator,
                    self.manager.vars[c3nodeVarName].var,
                    self.manager.vars[c3nodeVarName].previous[0],
                    threshold,
                    trgr
                )

                if response is None:
                    response = False
                self.response = response
                return

            except (IndexError):
                pass

    class DictTrigger:

        def __init__(self,
                     manager,
                     c3nodeVarName: str,
                     operator: str,
                     threshold: any,
                     repeat: bool = False,
                     key: str = None
                     ):

            self.manager = manager
            self.response = False
            self.dict = {}

            self._create_dict_trigger(
                        c3nodeVarName,
                        operator,
                        threshold,
                        repeat,
                        key)

        def __bool__(self):
            return self.response

        def __str__(self):
            return str(self.dict)

        def __getitem__(self, key):
            return self.dict[key]

        def _create_dict_trigger(
                    self,
                    c3nodeVarName,
                    operator,
                    threshold,
                    repeat,
                    key
                    ):

            if self.manager.vars[c3nodeVarName] is None:
                return

            trigger_id = f"{c3nodeVarName}_{operator}_"\
                         f"{threshold}_{repeat}_{key}"

            # If this is the first time calling the trigger, then add
            # it to the dictionary of triggers
            if trigger_id not in self.manager.triggers.keys():
                trgr = TriggerManager.Trigger(trigger_id,
                                              self.manager.vars[c3nodeVarName],
                                              threshold,
                                              operator,
                                              repeat)
                self.manager.triggers[trigger_id] = trgr

            # evaluate the trigger
            trgr: TriggerManager.Trigger = self.manager.triggers[trigger_id]

            # If the trigger has already fired and it's not a repeat trigger
            if (trgr.triggered) and (trgr.repeat is False):
                self.response = False
                self.dict = {}
                return

            operators = {
                         '>': [
                                 lambda x, z: x > z,
                                 ],
                         '>=': [
                                 lambda x, z: x >= z,
                                 ],
                         '<': [
                                 lambda x, z: x < z,
                                 ],
                         '<=': [
                                 lambda x, z: x <= z,
                                 ],
                         '==': [
                                 lambda x, z: x == z,
                                 ],
                         '!=': [
                                 lambda x, z: x != z,
                                 ],
                         '->=': [
                                 lambda x, z: x == z,
                                 lambda y, z: y != z,
                                 ],
                         '=->': [
                                 lambda x, z: x != z,
                                 lambda y, z: y == z,
                                 ],
                         'h->l': [
                                  lambda x, z: x <= z,
                                  lambda y, z: y > z,
                                  ],
                         'l->h': [
                                  lambda x, z: x >= z,
                                  lambda y, z: y < z,
                                  ]}

            def evaluate_conditions(op, x, y, z, key, trgr):

                try:

                    if self.manager.vars[c3nodeVarName].previous[0] is None:
                        return False, {}

                    sub_dict = {}

                    if len(operators[op]) == 2:

                        op1 = operators[op][0]
                        op2 = operators[op][1]

                        for k, v in x.items():
                            if k not in y[0].keys():
                                return False, {}
                            else:
                                # print(f"key: {key}, "
                                #       f"v: {v}, "
                                #       f"z: {z}")
                                if (key is not None):
                                    if v[key] is not None and y[0][k][key] is not None:
                                        # print(f"1: {v[key]}, 2: {y[0][k][key]}")
                                        if (op1(v[key], z) and
                                                op2(y[0][k][key], z)):
                                            sub_dict[k] = v
                                else:
                                    if op1(v, z) and op2(y[0][k], z):
                                        sub_dict[k] = v

                        if sub_dict == {}:
                            return False, {}
                        else:
                            if not trgr.repeat:
                                trgr.triggered = True
                            return True, sub_dict

                    else:

                        op1 = operators[op][0]

                        for k, v in x.items():
                            if k not in y[0].keys():
                                return False, {}
                            else:
                                if key is not None:
                                    if op1(v[key], z):
                                        sub_dict[k] = v
                                else:
                                    if op1(v, z):
                                        sub_dict[k] = v

                        if sub_dict == {}:
                            return False, {}
                        else:
                            if not trgr.repeat:
                                trgr.triggered = True
                            return True, sub_dict

                except (KeyError, AttributeError):
                    pass

            try:

                if self.manager.vars[c3nodeVarName].dict is None:
                    self.response = False
                    self.dict = None
                    return False, {}

                response, dict = evaluate_conditions(
                                operator,
                                self.manager.vars[c3nodeVarName].dict,
                                self.manager.vars[c3nodeVarName].previous,
                                threshold,
                                key,
                                trgr)

                if response is None:
                    response = False
                if dict is None:
                    dict = {}
                self.response = response
                self.dict = dict
                return

            except (IndexError):
                pass
    # ################## C3Node Variables ####################

    class C3NodeScalarVar:

        def __init__(self,
                     triggerMgr,
                     c3NodeScalarVar_name: str,
                     value=None
                     ):
            self._var = value
            self.previous = []
            self.lock = threading.Lock()

            triggerMgr.vars[c3NodeScalarVar_name] = self

        # Allows you to call the object as scalar_var() and return .var
        def __call__(self):
            with self.lock:
                return self._var

        # When you call the object to make it a string, this is what it returns
        def __str__(self):
            with self.lock:
                return str(self._var)

        # creates var as a property so that you can customize it's setter
        @property
        def var(self):
            return self._var

        # When setting the var parameter, run the update as well
        @var.setter
        def var(self, value):
            self.update(value)

        # Allows you to explicitly run the update 
        def update(self, value):
            with self.lock:
                self._update(value)

        def _update(self, value):
            self.previous.insert(0, self._var)
            if len(self.previous) >= 6:
                self.previous = self.previous[:5]
            self._var = value

    class C3NodeDictVar:

        def __init__(self,
                     triggerMgr,
                     c3NodeDictVar_name: str,
                     dict={}
                     ):
            self._dict = dict
            self.previous = []
            self.lock = threading.Lock()

            triggerMgr.vars[c3NodeDictVar_name] = self

        @property
        def dict(self):
            return self._dict

        @dict.setter
        def dict(self, dict):
            self.update(dict)

        # Allows you to explicitly run the update 
        def update(self, dict):
            with self.lock:
                self._update(dict)

        def _update(self, dict):
            self.previous.insert(0, self._dict)
            if len(self.previous) >= 6:
                self.previous = self.previous[:5]
            self._dict = dict

    class C3NodeMessageVar:

        def __init__(self,
                     triggerMgr,
                     c3NodeMessageVar_name: str,
                     message_name: str = None,
                     group_ids: list[str] = None):

            # Used if C3NodeVar is referencing received agent_status message
            self.msg_dict = {'agents': {}, 'timestamp': None}
            self.previous = {}
            # self.estimated = {}
            self.group_id_restriction = group_ids
            self.message_name = message_name
            self.lock = threading.Lock()

            triggerMgr.vars[c3NodeMessageVar_name] = self

        def update_c3node_var_by_c3m(self, c3node_var_key: str, c3Message):

            with self.lock:
                # print(f"In lock: {c3Message.message}")
                update = False
                restrict = self.group_id_restriction
                if restrict is not None:
                    if (c3Message.sender in restrict):
                        update = True
                else:
                    update = True

                if update:
                    for key, value in c3Message.message.items():
                        if key == self.message_name:
                            agent = c3Message.sender
                            if agent not in self.previous.keys():
                                self.previous[agent] = []
                            try:
                                # print("Here")
                                self.previous[agent].insert(
                                    0, self.msg_dict['agents'][agent])
                                if len(self.previous[agent]) >= 6:
                                    self.previous[agent] = \
                                        self.previous[agent][:5]
                            except KeyError:
                                pass
                            self.msg_dict['agents'][agent] = value
                            # self.msg_dict['timestamp'] = c3Message.timestamp
                            self.msg_dict['timestamp'] = \
                                datetime.now().strftime(
                                    "%m/%d/%Y, %H:%M:%S.%f"
                                )
                            # 08/19/2024, 22:01:07.235319
                            # print(f"C3Message: {c3Message.message}")
                    # print(f"CURR: {agent} - {self.msg_dict}")
                    # print(f"PREV: {agent} - {self.previous}")

    class C3NodePositionVar(C3NodeMessageVar):

        def estimated_position(self):

            def update_position(lat, lon, alt, relative_alt, vx, vy, vz, dt):

                R = 637_100_000  # Earth's radius in cm

                # Convert velocity to distance
                delta_n = float(vx) * float(dt)
                delta_e = float(vy) * float(dt)
                delta_z = float(vz) * dt

                # Convert distance to changes in latitude and longitude
                delta_lat = (delta_n / R) * (180 / math.pi)
                delta_lon = (delta_e / (R * math.cos(math.radians(lat)))) * \
                    (180 / math.pi)
                # print(f"delta: {delta_lat}, {delta_lon}", flush=True)

                # Calculate new latitude and longitude
                new_lat = lat + delta_lat
                new_lon = lon + delta_lon

                new_alt = alt - delta_z * 100
                new_relative_alt = relative_alt - delta_z / 100

                # print(f"d_x_y: {new_lat}, {new_lon}")
                return new_lat, new_lon, new_alt, new_relative_alt

            try:

                now = datetime.now()
                current = datetime.strptime(self.msg_dict['timestamp'],
                                            "%m/%d/%Y, %H:%M:%S.%f")
                delta = (now - current).seconds + \
                    (now - current).microseconds / 1_000_000

                estimated_pos = {}

                for agent, value in self.msg_dict['agents'].items():

                    cur_lat = value['lat']
                    cur_lon = value['lon']
                    cur_vx = value['vx']
                    cur_vy = value['vy']
                    cur_vz = value['vz']
                    cur_alt = value['alt']
                    cur_rel_alt = value['relative_alt']
                    cur_hdg = value['hdg']
                    cur_hdg_rad = value['hdg_rad']

                    new_lat, new_lon, new_alt, new_relative_alt = \
                        update_position(cur_lat,
                                        cur_lon,
                                        cur_alt,
                                        cur_rel_alt,
                                        cur_vx,
                                        cur_vy,
                                        cur_vz,
                                        delta)

                    estimated_pos[agent] = {
                        # 'lat': new_lat,
                        # 'lon': new_lon,
                        'lat': cur_lat,
                        'lon': cur_lon,
                        'vx': cur_vx,
                        'vy': cur_vy,
                        'vz': cur_vz,
                        'alt': cur_alt,
                        'relative_alt': cur_rel_alt,
                        # 'alt': new_alt,
                        # 'relative_alt': new_relative_alt,
                        'hdg': cur_hdg,
                        'hdg_rad': cur_hdg_rad
                    }

                return {'agents': estimated_pos, 
                        'timestamp': now.strftime("%m/%d/%Y, %H:%M:%S.%f")}

            except TypeError:
                pass


    # def dict_trigger(self,
    #                  c3nodeVarName: str,
    #                  threshold: any,
    #                  operator: str,
    #                  repeat: bool = False):

    #     if self.vars[c3nodeVarName] is None:
    #         return

    #     trigger_id = f"{c3nodeVarName}_{threshold}_"\
    #                  f"{operator}_{repeat}"

    #     # If this is the first time calling the trigger, then add
    #     # it to the dictionary of triggers
    #     if trigger_id not in self.triggers.keys():
    #         trgr = TriggerManager.Trigger(trigger_id,
    #                                       self.vars[c3nodeVarName],
    #                                       threshold,
    #                                       operator,
    #                                       repeat)
    #         self.triggers[trigger_id] = trgr
    #         # print(f"trigger_id: {trigger_id}")

    #     # evaluate the trigger
    #     trgr: TriggerManager.Trigger = self.triggers[trigger_id]

    #     # If the trigger has already fired and it's not a repeat trigger
    #     if (trgr.triggered) and (trgr.repeat is False):
    #         return False, None

    #     try:

    #         if self.vars[c3nodeVarName].dict is None:
    #             return False, {}

    #         if operator == '<=':
    #             sub_dict = {k: v for k, v in
    #                         self.vars[c3nodeVarName].dict.items()
    #                         if v <= threshold}
    #             if sub_dict != {}:
    #                 return True, sub_dict
    #             else:
    #                 return False, {}

    #         elif operator == '>=':
    #             sub_dict = {k: v for k, v in
    #                         self.vars[c3nodeVarName].dict.items()
    #                         if v >= threshold}
    #             if sub_dict != {}:
    #                 return True, sub_dict
    #             else:
    #                 return False, {}

    #         elif operator == '==':
    #             sub_dict = {k: v for k, v in
    #                         self.vars[c3nodeVarName].dict.items()
    #                         if v == threshold}
    #             if sub_dict != {}:
    #                 return True, sub_dict
    #             else:
    #                 return False, {}

    #         elif operator == '->=':

    #             if self.vars[c3nodeVarName].previous[0] is None:
    #                 return False, {}

    #             sub_dict = {}
    #             for k, v in self.vars[c3nodeVarName].dict.items():
    #                 if k not in self.vars[c3nodeVarName].previous[0].keys():
    #                     return False, {}
    #                 else:
    #                     if ((v == threshold) and
    #                             (self.vars[c3nodeVarName].previous[0][k] !=
    #                              threshold)):
    #                         sub_dict[k] = v

    #             if sub_dict == {}:
    #                 return False, {}
    #             else:
    #                 if not trgr.repeat:
    #                     trgr.triggered = True
    #                 return True, sub_dict

    #         elif operator == '=->':

    #             if self.vars[c3nodeVarName].previous[0] is None:
    #                 return False, {}

    #             sub_dict = {}
    #             for k, v in self.vars[c3nodeVarName].dict.items():
    #                 if k not in self.vars[c3nodeVarName].previous[0].keys():
    #                     return False, {}
    #                 else:
    #                     if ((v != threshold) and
    #                             (self.vars[c3nodeVarName].previous[0][k] ==
    #                              threshold)):
    #                         sub_dict[k] = v

    #             if sub_dict == {}:
    #                 return False, {}
    #             else:
    #                 if not trgr.repeat:
    #                     trgr.triggered = True
    #                 return True, sub_dict

    #         elif operator == 'l->h':

    #             if self.vars[c3nodeVarName].previous[0] is None:
    #                 return False, {}

    #             sub_dict = {}
    #             for k, v in self.vars[c3nodeVarName].dict.items():
    #                 if k not in self.vars[c3nodeVarName].previous[0].keys():
    #                     return False, {}
    #                 else:
    #                     if ((v >= threshold) and
    #                             (self.vars[c3nodeVarName].previous[0][k] <
    #                              threshold)):
    #                         sub_dict[k] = v

    #             if sub_dict == {}:
    #                 return False, {}
    #             else:
    #                 if not trgr.repeat:
    #                     trgr.triggered = True
    #                 return True, sub_dict

    #         elif operator == 'h->l':
    #             pass

    #     except TypeError:
    #         return
