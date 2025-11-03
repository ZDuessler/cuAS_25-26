# import time  # noqa: F401
# import math  # noqa: F401
# import random  # noqa: F401
from classes.c3_node import C3Node
from classes.c3_node_message import C3NodeMessage
from my_agent import Agent1  # noqa: F401
# from classes.trigger import Triggers
from classes.trigger import TriggerManager  # noqa: F401
from classes.agentutils import AgentUtils  # noqa: F401
from classes.c3_node_utils import C3NodeUtils  # noqa: F401

# import tracemalloc


class Name_Your_C3_Node(C3Node):
    '''
    Describe your class here
    '''

    def establish_logic_objects(self):

        # ### Initialize Logic Objects

        self.triggerMgr.C3NodePositionVar(self.triggerMgr,
                                          "agentPos",
                                          "agent_position",
                                          ["80001", "80002"])

        self.triggerMgr.C3NodeMessageVar(self.triggerMgr,
                                         "fleetArm",
                                         "arm_state",
                                         ["80001", "80002", "80003"])

        # ###########################################################

        self.agent_distance = self.triggerMgr.C3NodeDictVar(
            self.triggerMgr, "distance")

        pass

    # ##############################################################
    # Manage the main c3Node's main loop
    def c3node_main_loop(self):

        # estimated_pos = self.triggerMgr.vars['agentPos'].estimated_position()

        self.agent_distance.update(C3NodeUtils.list_of_distances(
            self.triggerMgr.vars['agentPos'].msg_dict
        ))

        # print(f"distances: {self.agent_distance.dict}", flush=True)

        distance_trigger = self.triggerMgr.DictTrigger(
            self.triggerMgr,
            c3nodeVarName='distance',
            operator='<',
            threshold=100,
            repeat=True
        )

        # print(f"dist: {self.agent_distance.dict}", flush=True)
        if distance_trigger:

            # print(f"dist_trig: {distance_trigger}")
            for key_agent_pair, value in distance_trigger.dict.items():
                agent1 = key_agent_pair.split('__')[0]
                agent2 = key_agent_pair.split('__')[1]

                self.send_direct_message(
                    agent1,
                    'Terminal',
                    {"connect_to": f'MISSION_{agent2[-1:]}'})
                self.send_direct_message(
                    agent2,
                    'Terminal',
                    {"connect_to": f'MISSION_{agent1[-1:]}'})

        pass

    # Manage the received messages from associated C3Nodes as C3Node
    def process_message_as_c3(self, c3Message):
        '''
        This function takes in all DIRECT and BROADCAST MESSAGES sent
        to this C3Node from agents.  You can process the message or pass
        it directly on using:
            - `self.send_direct_messge()`
            - `self.send_broadcast_messge()`

        '''

        # ##############################################

        # Run the default c3Node process
        super().process_message_as_c3(c3Message)

    # ##############################################################
    # Process the recieved message from associated C3Nodes as agent
    def process_message_as_agent(self, c3Message: C3NodeMessage):
        '''
        This function takes in messages as if this node is an agent
        in a C3 network meaning it is a subscriber or is being addressed
        directly from another agent
        '''

        print(f"The received message = {c3Message.message}")

        pass

        # Run the default c3Node process
        super().process_message_as_agent(c3Message)

    # ##############################################################
    # Manage the terminal inputs
    def process_terminal_input(self, terminal_input: str):
        '''
        This function processes any inputs into the terminal.  If you want
        to block other messages from printing to the terminal while you are
        typing to the terminal, use self.tprint(), otherwise print() will
        write write immediately to the terminal.
        '''

        # Run the default c3Node process which checks if the terminal
        # command starts with 'cd' (command direct) or 'cb' (command broadcast)
        super().process_terminal_input(terminal_input)

    # ##############################################################
    # Manage the broadcast message thread's loop
    # def c3_broadcast_loop(self):
    #     '''
    #     This function is a repeating broadcast function through
    #     the publisher mechanism.  All subscribers to this C3Node
    #     will get the message
    #     '''

    #     self.send_broadcast_message('ALL_SUBSCRIBERS',
    #                                           self.identity,
    #                                           "Do Something")

    # ################## Custom Definitions #######################


if __name__ == "__main__":
    c3TerminalNode = Name_Your_C3_Node(config_file='C3Terminal.yaml')
    c3TerminalNode.start()

    agent_count = 1
    agent_dict = {}
    for n in range(0, agent_count):
        agent_dict[f"agent[{n+1}]"] = \
            Agent1(loop_delay=0,
                   verbose=False,
                   configuration_file=f"ag_config_{n+1}.yaml")
