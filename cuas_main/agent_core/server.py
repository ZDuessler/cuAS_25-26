import sys
import time
import multiprocessing
from multiprocessing.sharedctypes import Synchronized
from multiprocessing.sharedctypes import SynchronizedArray
import threading
from typing import Tuple

class setInterval:
    def __init__(self, interval: float, action):
        self.interval = interval
        self.action = action
        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.__setInterval)
        self.thread.start()

    def __setInterval(self):
        nextTime = time.time() + self.interval
        while not self.stopEvent.wait(nextTime-time.time()):
            nextTime += self.interval
            if state.value != 4:
                self.action()
                inter.cancel()

    def cancel(self):
        self.stopEvent.set()

standalone: bool = False
dims: SynchronizedArray = multiprocessing.Array('d', 6)
conf: SynchronizedArray = multiprocessing.Array('d', 2)
state: Synchronized = multiprocessing.Value('i')
run: Synchronized = multiprocessing.Value('i')

def internal_runner(dims: SynchronizedArray, conf: SynchronizedArray, state: Synchronized, run: Synchronized):
    import socket
    import json

    server: socket.socket = None

    try:
        HOST = '127.0.0.1'
        PORT = 1337

        def conn_kill(conx: socket.socket):
            "Kill server or connection"
            conx.shutdown(socket.SHUT_RDWR)
            conx.close()

        while run.value == 1: # While spinning
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create server
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Make address/port reusable
            server.bind((HOST, PORT)) # Bind to 127.0.0.1:1337
            server.listen(1) # Listen for connection
            print(f"Listening on {HOST}:{PORT}")
            state.value = 4 # IPC is disconnected
            try:
                connection = server.accept()[0] # Wait for connection
                state.value = 3 # Assume no detection
                print("Accepted socket connection")
                while run.value == 1: # While spinning
                    data = connection.recv(1024) # Wait for data
                    if data: # If valid packet
                        msg = json.loads(data) # Parse message
                        for i in range(4):
                            dims[i] = msg[i] # Read values
                        dims[4] = dims[0] + dims[2]/2 # Find centroid
                        dims[5] = dims[1] + dims[3]/2
                        conf[0] = msg[4] # Read confidence
                        conf[1] = msg[5]
                        state.value = msg[6] # Set state to detector's value
                        
                        if standalone:
                            printOut()
                    else:
                        print("TCP ICP error")
                        connection = None
                        break # Exit loop to reset connection
            except:
                print("TCP IPC connection closed")
                conn_kill(connection)
                conn_kill(server)
        raise InterruptedError
    except InterruptedError or KeyboardInterrupt:
        if server:
            conn_kill(server)
    finally:
        sys.exit(0)

def printOut():
    "Print values out to console"
    print(list(dims), list(conf), state.value)
    
def getDims() -> Tuple[float, float, float, float, float, float]:
    "Returns the dimensions"
    return tuple(dims)

def getConf() -> Tuple[float, float]:
    "Returns the current and rolling average confidence"
    return tuple(conf)

def getStatus() -> int:
    """
    Returns detector state. 0 means active track,
    1 is intermittent track, 2 indicates a stale track,
    3 means the track is lost, and 4 only occurs when
    the detector isn't connected.
    """
    return state.value

def isConnected():
    "Checks if the detector is connected"
    return False if state.value == 4 else True

def noop():
    pass

def common_handler():
    global run
    run.value = 0
    p1.terminate()
    try:
        inter.cancel()
        p1.kill()
    finally:
        p1.join()
        p1.close()

def unix_handler(sig, frame):
    common_handler()
    sys.exit(0)

def win32_handler(a):
    common_handler()
    sys.exit(0)

def start(callback=noop):
    """
    Starts the server.
    A callback that runs once connected may be specified but is optional.
    """
    global inter, p1, run
    
    run.value = 1
    p1 = multiprocessing.Process(None, internal_runner, None, (dims, conf, state, run), daemon=True)
    inter = setInterval(.2, callback)

    if sys.platform == "win32":
        import win32api
        win32api.SetConsoleCtrlHandler(win32_handler, True)
    else:
        import signal
        signal.signal(signal.SIGINT, unix_handler)
        
    p1.start()

if __name__ == "__main__":
    standalone = True
    start()
    p1.join()