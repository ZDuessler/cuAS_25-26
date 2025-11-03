# C3

#### 2024-'25 authors: Junhyung Park, Gunnar Gott

The C3 files comprise Agent Core proper. That is, they contain the stuff that is actually Agent Core, as opposed to the handful of plugins that I (Geoffrey) wrote which handle interfacing with various external non-AC programs.

## [`C3Mission_1.py`](/agent_core/C3Mission_1.py)

C3Mission, executed on the ground station, forms the bulk of the logic for making the cUAS system actually do stuff. This is where the state machine and logic for reacting to data inputs reside. This file should be executed on the ground station.

## [`C3Terminal.py`](/agent_core/C3Terminal.py)

C3Terminal is executed on the drones to provide a connection to MavProxy on the drone side. It also allows for some manual operation.

# AgentCore Plugins

#### 2024-'25 author: Geoffrey Stentiford

There are three plugins which run inside C3Mission:
- [`server.py`](server.md), a TCP IPC server listening on `127.0.0.1:1337` which receives data from the camera-based detector
- [`radar.py`](radar.md), a WebSocket client that streams radar data from the relay script running on the Fortem laptop
- [`data_bridge.py`](data_bridge.md), a UNIX socket IPC server listening on `/tmp/ac_bridge` which sends data to the GUI

The three links above link to each plugin's documentation.

# Other files

#### Authors: the contractors

## `my_agent`

`my_agent` is the main Agent Core library. Do not delete.

## Log files
The sundry CSV files are position logs. They are overwritten by C3Mission.

## Config files
`.parm` and `.yaml` files configure Agent Core and MavLink.

## Subfolders and everything else
There be dragons.