import sys
import multiprocessing
import os
from multiprocessing.sharedctypes import SynchronizedArray
from multiprocessing.sharedctypes import Synchronized
from enum import Enum

SOCKET = "/tmp/ac_bridge"

class Status(Enum):
    NONE = 0 # 0,0
    STARTED = 1 # 0,1
    READY = 2 # 1,0
    BUSY = 3 # 1,1

# Shared with child process
dVals: SynchronizedArray = multiprocessing.Array('d', 6)
rVals: SynchronizedArray = multiprocessing.Array('d', 5)
mode: Synchronized = multiprocessing.Value('i')
state: SynchronizedArray = multiprocessing.Array('i', 3)

state[0] = 0 # connection down
state[1] = 0 # not busy
state[2] = 1 # spin

def internal_runner(dVals: SynchronizedArray, rVals: SynchronizedArray, mode: Synchronized,state: SynchronizedArray, path: str):
    '''
    Runs on child process to send data over IPC
    '''
    import socket
    import json

    def sock_kill(conx: socket.socket):
        '''
        Kills the server
        '''
        conx.shutdown(socket.SHUT_RDWR)
        conx.close()

    try:
        state[0] = 0 # connection down
        state[1] = 1 # module busy

        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) # Create the server
        server.bind(path) # Attach to socket file
        server.listen(1) # Listen for connection
        print(f"Listening on {path}")

        # try:
        while state[2] == 1: # while spinning
            connection = server.accept()[0] # Accept incoming
            data = connection.recv(1024) # Wait for message
            if data: # If client sent data (just a UTF-8 "0")
                print(f"Client attached to socket {path}")
                state[0] = 1 # connection up
                state[1] = 0 # not busy
                while state[0] == 1: # break if connection down
                    if state[1] == 1: # if busy (data to send)
                        # dVals[0] is actually the timestamp, so we slice that off
                        payload = json.dumps((tuple(dVals)[1:6], tuple(rVals), float(dVals[0]), mode.value)) # Encode to JSON
                        try:
                            connection.sendall(payload.encode()) # Send data
                        except:
                            state[0] = 0 # connection lost
                            print("UNIX socket IPC connection lost")
                        state[1] = 0 # not busy
    except KeyboardInterrupt or InterruptedError:
        sock_kill(server) # Shutdown if Ctrl+C or SIGTERM
    finally:
        sys.exit(0)

def pushData(disco: tuple[float], rogue: tuple[float], status: str):
    '''
    Push values to IPC
    '''
    while state[1] == 1 and state[0] == 1:
        pass # wait for not busy
    if state[0] == 1 and state[1] == 0:
        match status: # Convert status to integer
            case "STABILIZE":
                mode.value = 1
            case "AUTO":
                mode.value = 0
            case "RTL":
                mode.value = 2
            case "LAND":
                mode.value = 3
        dVals[0] = disco[0]
        dVals[1] = disco[1]
        dVals[2] = disco[2]
        dVals[3] = disco[3]
        dVals[4] = disco[4]
        dVals[5] = disco[5]
        rVals[0] = rogue[0]
        rVals[1] = rogue[1]
        rVals[2] = rogue[2]
        rVals[3] = rogue[3]
        rVals[4] = rogue[4]
        state[1] = 1

def getStatus():
    '''
    Get the connection status of the server
    '''
    if state[0] == 1:
        if state[1] == 0:
            return Status.READY
        return Status.BUSY
    if state[1] == 1:
        return Status.STARTED
    return Status.NONE 

def unix_handler(sig, frame):
    '''
    Handle SIGTERM
    '''
    state[2] = 0 # stop
    state[0] = 0 # connection down
    state[1] = 0 # not busy
    p1.terminate() # Ask nicely
    try:
        p1.kill() # Force termination
    finally:
        p1.join() # Wait for shutdown
        p1.close() # Release resources
    os.unlink(SOCKET) # Delete the socket
    sys.exit(0)

def start():
    '''
    Start the IPC Server
    '''
    import signal
    global p1
    
    # Ensure old socket file is gone
    try:
        os.unlink(SOCKET)
    except OSError: # If old socket is there but somehow not removable
        if os.path.exists(SOCKET):
            print("\n***Can't remove old socket! Continuing but expect issues.***\n")
    
    # Launch child process
    p1 = multiprocessing.Process(None, internal_runner, None, (dVals, rVals, mode, state, SOCKET), daemon=True)

    signal.signal(signal.SIGINT, unix_handler) # Hook exit signal
        
    p1.start()

if __name__ == "__main__":
    start()
    p1.join()