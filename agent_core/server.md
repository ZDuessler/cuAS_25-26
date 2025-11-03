# IPC-over-TCP Server

This plugin lets C3Mission read values from the TensorFlow.js detector app running on Electron. It launches a child process to listen for TCP connections on `127.0.0.1:1337` and parse messages received over that port.

## Standalone usage

From terminal:  
```bash
python server.py
```

## API Calls

```python
# Import library
import server

# Start server
server.start()

# Get dimensions
server.getDims()

# Get confidence
server.getConf()

# Check for detector connection
server.isConnected()
```

## Data Format

The detector operates on a 300x300 input. As such, all output dimensions are scaled to 300x300, and (0,0) is not the center of that 300x300 screen, but the top left corner. The X axis increases rightwards and the Y axis increases downwards.  

 Dimensions are given as a tuple arranged as `(x1, y1 w, h, x2, y2)` where:  
 -  `x1` is measured from left edge of the 300x300 screen to the left edge of the bounding box,
 - `y1` is measured from the top edge of the 300x300 screen to the top edge of the bounding box,
 - `w` and `h` represent the width and height of the bounding box, and
 - `x2` and `y2` are the center of the bounding box, again measured from the top-left corner of the screen.

The confidence is given as a tuple consisting of two `floats`, with the first value representing the most current confidence (decimal out of 1.0) and the second value representing the five-iteration rolling average of the confidence.

## Internals

The server expects a JSON string over its network port representing this JavaScript data structure:
```javascript
[x, y, w, h, curr_conf, avg_conf, status]
```

The reason a TCP socket was chosen instead of a UNIX socket was because both the server and detector were originally developed on Windows, and then we simply didn't bother changing what worked well enough.
