import websocket
import json
from typing import List, Dict, TypedDict, Tuple
import time
import numpy as np
import threading
import math

# Certainties of a track being of that type
Probs = TypedDict('Probs', {
    "other": float,
    "bird": float,
    "dronerotor": float,
    "droneplane": float,
    "helicopter": float,
    "airplane": float
})

Detection = TypedDict('Track', {
    "id": str,
    "rcs": float,
    "start": int,
    "heading": float,
    "isStationary": bool,
    "lla": List[float],
    "category": str,
    "catProb": Probs
})

class Track:
    "Object representing a specific track at one moment in time"
    def __init__(self, detect: Detection):
        self.id = detect["id"]
        self.rcs = detect["rcs"]
        self.start = detect["start"]
        self.lla = detect["lla"]
        self.heading = detect["heading"]
        self.isStationary = detect["isStationary"]
        self.category = detect["category"]
        self.catProb = detect["catProb"]
        self.raw = detect
        self.idx = -1
        self.current = True

    def atLLA(self, lla: List[float], tol: List[float]):
        '''
        Checks if this track is at the specified lat/long to within
        the specified tolerance.
        '''
        for idx, val in enumerate(self.lla):
            if abs(lla[idx] - val) > tol[idx]:
                return False
        return True
    
    def checkFilter(self, age: int = 0, rcs: float = 0.0,
                    matchCat: List[str] = ["other", "bird", "dronerotor", "droneplane", "helicopter", "airplane"],
                    minCert: float = 0.0):
        "Checks if this track would be included under the filter criteria"
        d = self.raw
        return False if ((server_time - d["start"]) < age or d["rcs"] < rcs or not (d["category"] in matchCat) or d["catProb"][d["category"]] < minCert) else True

standalone : bool = False
server_time: int = 0
detections : List[Detection] = list()
tracks : List[Track] = list()
catalog : Dict[str, int] = dict()
is_open : bool = False
NUL = Track({"id": "NUL", "rcs": -1.0, "start": -1, "lla": [np.nan, np.nan, np.nan],
             "heading": math.nan, "isStationary": True, "category": "other",
             "catProb": {"other": -1.0, "bird": -1.0, "dronerotor": -1.0,
                            "droneplane": -1.0, "helicopter": -1.0, "airplane": -1.0}})

def handler(ws: websocket.WebSocketApp, message: str):
    "Handles incoming WebSocket messages. Do not call manually."
    global server_time, detections, catalog, tracks
    parsed: Tuple[List[Detection] | str, int, int, Dict[str, int]] = tuple(json.loads(message))
    server_time = parsed[2] # Update time from server
    if parsed[1] == 0: # If not error
        detections = parsed[0] # Update detections
        catalog = parsed[3] # Update catalog
        for inst in tracks: # Mark all old tracks as non-current
            inst.current = False
        tracks.clear() # Clear old track references
        for detection in detections: # Create new track objects
            tracks.append(Track(detection))
    else:
        print("Error: " + str(parsed[0]))

def interactive(ws: websocket.WebSocketApp, message: str):
    "Prints received data when running in standalone mode. Do not call manually."
    global server_time
    parsed: Tuple[List[Detection] | str, int, int, Dict[str, int]] = tuple(json.loads(message))
    server_time = parsed[2]
    print(parsed[0])
    print(parsed[3])
    print("Time diff: " + str(int(time.time() * 1000) - server_time) + "\n")

def numTracks():
    "Get the number of tracks"
    global detections
    return len(detections)

def getAll():
    "Get all tracks"
    # global detections
    # return detections
    global tracks
    return tracks

def getByNum(idx: int):
    """
    Get track by index value
    (likely won't be too useful to you)
    """
    global detections, tracks
    if (idx + 1) > len(detections):
        return tracks[idx]
    return NUL

def getById(id: str):
    "Get track by its ID string"
    global tracks, catalog
    if id in catalog:
        return tracks[int(catalog[id])]
    # if len(detections) > 0:
    #     for det in detections:
    #         if det["id"] == id:
    #             return det
    return NUL

def filterTracks(age: int = 0, rcs: float = 0.0, id: str = "NUL",
                 matchCat: List[str] = ["other", "bird", "dronerotor", "droneplane", "helicopter", "airplane"],
                 minCert: float = 0.0):
    "Includes and excludes tracks based on criteria"
    global detections, tracks
    filtered: List[Track] = []
    if len(detections) > 0:
        for idx, det in enumerate(detections):
            if (server_time - det["start"]) >= age and det["rcs"] >= rcs and det["id"] != id and det["category"] in matchCat and det["catProb"][det["category"]] >= minCert:
                filtered.append(tracks[idx])
    return filtered

def getByLLA(lla: List[float], tol: List[float]):
    """
    Returns first track found to be at the specified lat/long to within
    the specified tolerance
    """
    global detections, tracks
    if len(detections) > 0:
        for idx, det in enumerate(detections):
            coords = np.array(det["lla"])
            target = np.array(lla)
            diff = np.abs(coords - target)
            if (diff <= np.array(tol)).all():
                return tracks[idx]
    return NUL

def excludeByLLA(lla: List[float], tol: List[float]):
    """
    Excludes all tracks found to be at the specified lat/long to within
    the specified tolerance
    """
    global detections, tracks
    filtered: List[Track] = []
    if len(detections) > 0:
        for idx, det in enumerate(detections):
            coords = np.array(det["lla"])
            target = np.array(lla)
            diff = np.abs(coords - target)
            if (diff > np.array(tol)).all():
                filtered.append(tracks[idx])
    return filtered

def isOpen():
    "Check if connection to relay server is up"
    return is_open

def on_error(ws: websocket.WebSocketApp, error):
    print(error)
    ws.close()

def on_close(ws: websocket.WebSocketApp, close_status_code: int, close_msg: str):
    global is_open
    is_open = False
    print("WebSocket connection closed: " + str(close_status_code) + " " + str(close_msg))

def on_open(ws: websocket.WebSocketApp):
    global is_open
    is_open = True
    print("WebSocket connection opened")

def connect(server: str = "127.0.0.1") -> bool:
    "Connects to the relay server"
    global standalone, is_open, ws, runner
    if is_open:
        print("Aborting connect() because socket is already open")
        return
    addr = "ws://" + server + ":8080"
    
    websocket.enableTrace(False)
    if standalone:
        msg_cb: function = interactive
    else:
        msg_cb: function = handler
    ws = websocket.WebSocketApp(addr, on_open=on_open,
                              on_message=msg_cb, on_error=on_error, on_close=on_close)

    def proxy():
        ws.run_forever(reconnect=1)

    runner = threading.Thread(target=proxy)
    runner.start()
    time.sleep(.5)
    return is_open
    
def kill():
    "Shuts down client"
    global is_open
    is_open = False
    ws.close()
    if runner.is_alive():
        runner.join()

if __name__ == "__main__":
    import sys
    standalone = True
    if len(sys.argv) > 1:
        connect(str(sys.argv[1])) 
    else:
        connect()