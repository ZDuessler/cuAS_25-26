import subprocess
import time
import os
import yaml

# kill $(pgrep -f sim_vehicle.py)
# screen -S <session_name> -X quit (or kill)
# killall screen
# screen -wipe
# screen -r <session-name> ... to view the session windows

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# sys.path.insert(0, parent_dir)

config_file_path = os.path.join(parent_dir, 'startup/startup.yaml')
with open(config_file_path, 'r') as file:
    config = yaml.safe_load(file)

# Define the session name
session_name = "sim_session"

# Step 1: Start a new screen session in detached mode
subprocess.run(['screen', '-dmS', session_name])

# Step 2: Send an echo command to the main screen (window 0) and wait 3 sec
subprocess.run(['screen', '-S', session_name, '-p', '0',
                '-X', 'stuff', 'echo "Hello from the main session screen"\n'])
time.sleep(3)

# Step 3: Add the sims
agent_count = 0
for sim in config['ONBOARD_SIMULATIONS']:
    subprocess.run(['screen', '-S', session_name, '-X',
                    'screen', '-t', f'subscreen{agent_count}'])
    command = ('python ~/AgentCore/SITL/ardupilot/Tools'
               '/autotest/sim_vehicle.py '
               f'--instance {agent_count} '
               f'--sysid {agent_count + 1} '
               '--no-mavproxy '
               '-v ArduCopter '
               f'-L {sim}')
    subprocess.Popen(['screen', '-S', session_name, '-p',
                      f'subscreen{agent_count}',
                      '-X', 'stuff', f'{command}\n'])
    agent_count += 1
    time.sleep(3)

# Step 4: Add the agents via the startup_agents.py script
time.sleep(3)
subprocess.run(['screen', '-S', session_name, '-X',
                'screen', '-t', 'auto_agents'])
startup_file_path = os.path.join(parent_dir, 'startup')
command = (f'python {startup_file_path}/startup_agents.py')
subprocess.Popen(['screen', '-S', session_name, '-p',
                  'auto_agents', '-X', 'stuff', f'{command}\n'])
