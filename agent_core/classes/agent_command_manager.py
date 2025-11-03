import os
from classes.commands.default_sys_commands import DefaultSysCommands
from classes.commands.default_nav_commands import DefaultNavCommands
from classes.commands.default_msn_commands import DefaultMsnCommands
# from agent_hub import AgentHub

os.environ['MAVLINK20'] = '1'


class AgentCommandManager(DefaultSysCommands,
                          DefaultNavCommands,
                          DefaultMsnCommands):

    '''
    This classs consoldates the default (sys, nav, msn) and provides a place to
    create custom commands.
    Instantiating this class will allow you to call any of the default commands
    as well as add your own here.
    ### IF YOU ADD CUSTOM COMMANDS, MAKE SURE THEY START WITH 'cmd_' ###
    '''

    def __init__(self,
                 config: dict,
                 agent_hub,
                 mav_connection,
                 verbose: bool = False):
        DefaultSysCommands.__init__(self, config, agent_hub,
                                    mav_connection, verbose)
        DefaultNavCommands.__init__(self, config, agent_hub,
                                    mav_connection, verbose)
        DefaultSysCommands.__init__(self, config, agent_hub,
                                    mav_connection, verbose)

        # self.ack_queue = agent_hub.mavlink_manager.current_mavlink_message_dict

    def cmd_custom_takeoff(self, veryfy=False) -> bool:
        print("Custom Takeoff Data")
        pass
