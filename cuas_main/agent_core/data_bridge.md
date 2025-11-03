# AgentCore-to-GUI Bridge

This plugin allows C3Mission to push data about both the discovery and the rogue drones to the GUI. It launches a child process to listen for a connection on `/tmp/ac_bridge`, waits for a handshake message, and sends messages containing values when the update flag is set.

## Standalone usage

```bash
python data_bridge.py
```
(It really doesn't do anything in standalone since there's no data to push. It accepts connections at that's it.)

## API Calls

```python
# Import the bridge
import data_bridge

# Start IPC server
data_bridge.start()

# Push data to GUI
data_bridge.pushData((time.time(), disco_lat, disco_long, disco_abs_alt, disco_rel_alt, disco_hdg), \
    (rogue_lat, rogue_long, rogue_abs_alt, rogue_rel_alt, rogue_hdg), \
    disco_status)

# Get bridge status
data_bridge.getStatus()
```

## Data Format

### `pushData()`

`pushData()` expects two tuples and a string.  

The first tuple contains, in this order:
1. current UNIX time
2. discovery drone latitude
3. discovery drone longitude
4. discovery drone absolute altitude
5. discovery drone relative altitude
6. discovery drone heading

The second tuple contains, in this order:
1. rogue drone latitude
2. rogue drone longitude
3. rogue drone absolute altitude
4. rogue drone relative altitude
5. rogue drone heading

Lastly, the string is the discovery drone's state (`discoStatus` in C3Mission).
Valid values are  `"AUTO"`, `"STABILIZE"`, `"RTL"`, and `"LAND"`, mapping to 0-3, respectively.

### `getStatus()`

`getStatus()` returns a value from the following `Enum`:
```python
class Status(Enum):
    NONE = 0    # not initialized
    STARTED = 1 # started but no connection
    READY = 2   # up and ready for data
    BUSY = 3    # up but busy sending data
```

From within `C3Mission_1.py`, you won't have to worry about checking if the bridge is busy before calling `pushData()`, since `pushData()` already does that for you.
The `BUSY` state also lasts for an incredibly short amount of time, so in practice you don't encounter it when using the library.

## Internals

The GUI is designed to run exclusively on the ground station in parallel to C3Mission. The two communicate using a UNIX socket over which data is passed as JSON.