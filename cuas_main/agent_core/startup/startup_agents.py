import yaml
import os
import sys

from my_agent import Agent1

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

config_file_path = os.path.join(parent_dir, 'startup/startup.yaml')
with open(config_file_path, 'r') as file:
    config = yaml.safe_load(file)

# print("################ Here I am #####################")

for agent in config['AGENTS']:
    agent_config_file_path = os.path.join(parent_dir, f'{agent}')
    agent_dict = {}
    agent_dict[f"{agent}"] = \
        Agent1(loop_delay=0,
               verbose=False,
               configuration_file=agent_config_file_path
               )
