from classes.agent_core import AgentCore
from agent_status_class import AgentStatus, AgentPosition  # noqa: F401
from classes.c3_node_message import C3NodeMessage
from classes.c3_node import AgentC3Node, AgentC3NodeManager  # noqa: F401

import inspect


class Agent1(AgentCore):
    '''
    The main agent:

    Class Variables:

        The following class vailables are available to use within the
        the instances of AgentCore.

        `self.acm (AgentCommandManager_v2)`: This is a reference to your
        AgentCommandManager object.  It's used to call commands
        `self.c3nm (AgentC3NodeManager)`: This is a reference to your
        AgentC3NodeManager object.  The following commands are available
        to use to communicate to the C3Node or other nodes in the network
                [C3NodeName].send_message(
                        message: str or dict):
                        -> sends message to C3Node
                [C3NodeName].send_direct_message(
                        message: str or dict,
                        ids: Union[str, List[str]] = None):
                        -> sends message directly to IDs in [ID(s)]
                [C3NodeName].send_broadcast_message(
                        message: str or dict,
                        message_type: str = None,
                        ids: str or List[str]] = None):
                        -> publishes message to subscribers of group(s)
                        in [group_name(s)]
        `self.agent_status (Agent_Status_v2)`: This is a reference to your
        AgentStatus object.

    Methods:
        - `agent_core_main_loop()`: This function overwrites the function
            from AgentCore.  It's the main thread function in AgentCore.
            YOUR MAIN AI LOGIC GOES HERE
        - `process_c3_input()`: This function gets run every
            run_agent_core_loop_thread() cycle in AgentCore.  It is used to
            process the C3 inputs from the C3 manager.
            YOUR RESPONSE TO EXTERNAL MESSAGES GOES HERE
    '''
    # # These are here for easier Intellisence referencing
    # inspect.getdoc(AgentStatus)      # self.agent_status
    # inspect.getdoc(AgentPosition)       # self.agent_status.agent_position
    # inspect.getdoc(AgentC3NodeManager)  # self.c3nm
    # inspect.getdoc(AgentC3Node)

    # ### Establish timers and conditionals:

    # Initialize the timers and conditionals used in your loop logic
    def establish_logic_objects(self):

        # def send_status_update():
        #     lat = self.agent_status.agent_position.to_dict()["lat"]
        #     lon = self.agent_status.agent_position.to_dict()["lon"]
        #     alt = self.agent_status.agent_position.to_dict()["alt"]
        #     rel_alt = \
        #         self.agent_status.agent_position.to_dict()["relative_alt"]
        #     self.c3nm.TERMINAL.send_direct_message(
        #         {"lat_lon_alt": {
        #             "lat": lat,
        #             "lon": lon,
        #             "alt": alt,
        #             "rel_alt": rel_alt
        #         }}, "TERMINAL")

        # self.time_keeper = TimeKeeper(1.0, send_status_update)
        # self.time_keeper = self.TimeKeeper(5, lambda: print(
        #         self.agent_status.agent_position.to_dict()["relative_alt"])
        #     )

        pass

    def agent_core_main_loop(self):

        # # if self.agent_status.has_new("arm_state"):
        # # OR
        # if self.agent_status.arm_state.is_new:
        #     if self.agent_status.arm_state.state == 'armed-above idle':
        #         self.c3nm.MISSION.send_direct_message(
        #             "arm_swarm")
        #         # OR
        #         # self.c3nm.MISSION.send_direct_message(
        #         #     "arm", "80002")
        #         # OR
        #         # self.c3nm.MISSION.send_direct_message(
        #         #     "cd[80002] launch2"
        #         # )

        # if self.agent_status.agent_position.is_new:
        # OR
        # if self.agent_status.has_new("agent_position"):
        #     # self.c3nm.MISSION.send_direct_message(
        #     #     self.agent_status.agent_position.to_dict(), "MISSION")
        #     print(f"hdg: {self.agent_status.agent_position.hdg}")

        if self.agent_status.flight_mode.is_new:

            # self.c3nm.MISSION.send_direct_message("arm_swarm")

            target_system = self.agent_hub.mavlink_manager.\
                      mav_connection.target_system
            print(f"The new flight mode "
                  f"{target_system}-"
                  f"{self.agent_status.agent_id}: "
                  f"{self.agent_status.flight_mode.mode}")

        return True

    # Manage commands from the C3 nodes
    def process_c3_input(self, c3_message: C3NodeMessage):
        '''
        Place all your responses to C3 traffic here.

        Params:
            `c3_message (C3NodeMessage)`: The most recent C3 message
             received by the AgentHub
        '''

        if c3_message.message[0:12] == "{\"connect_to":
            mission = f"MISSION_{self.config['SYS_ID']}"
            getattr(self.c3nm, mission).send_direct_message(
                c3_message.message
            )
            return

        super().process_c3_input(c3_message)


class Agent2(AgentCore):
    '''
    The main agent:

    Class Variables:
        `self.acm (AgentCommandManager_v2)`: This is a reference to your
        AgentCommandManager object
        `self.c3nm (AgentC3NodeManager)`: This is a reference to your
        AgentC3NodeManager object
                [C3NodeName].send_direct_message(
                        message: str or dict,
                        message_type: str = None,
                        ids: Union[str, List[str]] = None)
                [C3NodeName].send_broadcast_message(
                        message: str or dict,
                        message_type: str = None,
                        ids: str or List[str]] = None)
                [C3NodeName].send_message(
                        message: str or dict)
        `self.agent_status (Agent_Status_v2)`: This is a reference to your
        AgentStatus object.

    Methods:
        - `agent_core_main_loop()`: This function overwrites the function
            from AgentCore.  It's the main thread function in AgentCore.
            YOUR MAIN AI LOGIC GOES HERE
        - `process_c3_input()`: This function gets run every
            run_agent_core_loop_thread() cycle in AgentCore.  It is used to
            process the C3 inputs from the C3 manager.
            YOUR RESPONSE TO EXTERNAL MESSAGES GOES HERE
    '''
    # These are here for easier Intellisence referencing
    inspect.getdoc(AgentStatus)      # self.agent_status
    inspect.getdoc(AgentPosition)       # self.agent_status.agent_position
    inspect.getdoc(AgentC3NodeManager)  # self.c3nm
    inspect.getdoc(AgentC3Node)

    def agent_core_main_loop(self):

        # if self.agent_status.agent_position.is_new:
        #     self.c3nm.MISSION.send_direct_message(
        #         self.agent_status.agent_position.to_dict())

        # if self.agent_status.arm_state.is_new:
        #     # print(f"80002's new arm state = "
        #     #       f"{self.agent_status.arm_state.state}")
        #     self.c3nm.MISSION.send_direct_message(
        #         # {"arm_state": self.agent_status.arm_state.to_dict()}
        #         self.agent_status.arm_state
        #         )

        # if self.agent_status.HEARTBEAT.is_new:
        #     print(f"I have a new heartbeat: "
        #           f"{self.agent_status.HEARTBEAT}")

        return True

    # Manage commands from the C3 nodes
    def process_c3_input(self, c3_message: C3NodeMessage):
        '''
        Place all your responses to C3 traffic here.  If you intercept
        and process the command here, ensure you 'return' before performing
        the super() at the end

        Params:
            `c3_message (C3NodeMessage)`: The most recent C3 message
             received by the AgentHub
        '''

        # print(f"Sender: {c3_message.sender} / Message: {c3_message.message}")

        # if c3_message.sender == 'MISSION' and c3_message.message == "launch":
        #     # print("I should be launching")
        #     self.acm.cmd_sys_mode_change("guided")
        #     self.acm.cmd_msn_upload_waypoints([
        #             self.acm.wpt.Takeoff(alt=10),
        #             self.acm.wpt.Waypoint(
        #                 delay=0,
        #                 lat=39.01738934103869,
        #                 lon=-104.89386727207446,
        #                 alt=0
        #             ),
        #             self.acm.wpt.SplineWaypoint(
        #                 delay=0,
        #                 lat=39.0173598694517,
        #                 lon=-104.89247895503722,
        #                 alt=0
        #             ),
        #             self.acm.wpt.ChangeSpeed(
        #                 speed=7,
        #                 speed_type=1
        #             ),
        #             self.acm.wpt.Waypoint(
        #                 delay=5,
        #                 lat=39.01881575116393,
        #                 lon=-104.89355622836666,
        #                 alt=0
        #             ),
        #             self.acm.wpt.Land()
        #         ])
        #     self.acm.cmd_sys_arm_disarm("arm", True, 20)
        #     self.acm.cmd_nav_guided_takeoff(8)
        #     time.sleep((3))
        #     self.acm.cmd_sys_mode_change("AUTO")

        # if c3_message.sender == '80001' and c3_message.message == "go":
        #     print("80002 Go to the point")
        #     self.acm.cmd_sys_mode_change("guided")
        #     self.acm.cmd_nav_guided_goto_pos_hat(
        #         39.0173633447192, -104.8931487838945)
        #     # self.acm.cmd_msn_upload_waypoints([self.acm.wpt.Waypoint(
        #     #     0, 39.0173633447192, -104.8931487838945, 0)])
        #     # time.sleep(1)
        #     # self.acm.cmd_msn_start_mission()

        # if the message is not managed above, AgentCore will look for
        # the command in the agent's COMMANDS: YAML dictionary
        super().process_c3_input(c3_message)


if __name__ == "__main__":

    agent1 = Agent1(loop_delay=0,
                    verbose=False,
                    configuration_file="ag_config_1.yaml")
    agent2 = Agent1(loop_delay=0,
                    verbose=False,
                    configuration_file="ag_config_2.yaml")
    agent3 = Agent1(loop_delay=0,
                    verbose=False,
                    configuration_file="ag_config_3.yaml")
