# ------------------------------------------
# C1C Junhyung Park, c/o 2025
# cUAS Capstone 24-25
# junhyung1821@gmail.com
# ------------------------------------------

import time  # noqa: F401
import math  # noqa: F401
import random  # noqa: F401
from datetime import datetime  # noqa: F401
import json
import math
from typing import Dict

from classes.c3_node import C3Node
from classes.c3_node_message import C3NodeMessage
from my_agent import Agent1
# from classes.trigger import T riggers
from classes.trigger import TriggerManager  # noqa: F401
from classes.agentutils import AgentUtils  # noqa: F401
from classes.c3_node_utils import C3NodeUtils  # noqa: F401

# C2 library
import sympy as sp
import numpy as np

# Data recording library
import os
import csv

# For detector and camera use
import server
server.start()
centerX = -1
centerY = -1
# import tracemalloc

# Radar Use
import radar

# State variables
from enum import Enum

import data_bridge
data_bridge.start()

class DiscoStates(Enum):
    # Locate by radar data
    LOCATE = 1 
    # Launch issue commands to launch 5m
    LAUNCH = 2
    # Follow the altitude of the rogue 3m under
    ALTITUDE_CONTROL = 3
    # Follow the coordinate of the rogue within x m
    COORD_CONTROL = 4
    # Use coordinate / altitude to center the rogue drone within camera FOV
    FUSION_CONTROL = 5
    # Solely use camera to follow rogue drone
    CAMERA_CONTROL = 6
    # Emergency
    EMERGENCY = 7
    # Yaw find
    YAW_FIND = 8
    # Scan
    SCAN = 9

class Agent_Mission_Node(C3Node):
    '''
    Describe your class here
    '''

    def establish_logic_objects(self):
        # Initialize Logic Objects

        # C3 Variables
        self.triggerMgr.C3NodePositionVar(self.triggerMgr,
                                          "agentPos",
                                          "agent_position",
                                          ["80001", "80002"])

        self.triggerMgr.C3NodeMessageVar(self.triggerMgr,
                                        "agentMode",
                                        "flight_mode",
                                        ["80001", "80002"])
        
        # Prevent file overwriting
        if os.path.exists(f"disco_position.csv"):
            pass
        else:
            fileDisco = open(f"disco_position.csv", 'w', newline='')
            fieldnames = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Relative Altitude', 'Heading', 'State']
            writer = csv.DictWriter(fileDisco, fieldnames=fieldnames)
            writer.writeheader()
            fileDisco.close()

        if os.path.exists(f"rogue_position.csv"):
            pass
        else:
            fileRogue = open(f"rogue_position.csv", 'w', newline='')
            fieldnames = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Relative Altitude', 'Heading']
            writer = csv.DictWriter(fileRogue, fieldnames=fieldnames)
            writer.writeheader()
            fileRogue.close()

        fileDisco = open(f"disco_position.csv", 'a', newline='')
        fieldnames = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Relative Altitude', 'Heading', 'State']
        writer = csv.DictWriter(fileDisco, fieldnames=fieldnames)
        writer.writerow({'Time': "BREAK", 'Latitude': datetime.today()})
        fileDisco.close()

        fileRogue = open(f"rogue_position.csv", 'a', newline='')
        fieldnames = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Relative Altitude', 'Heading']
        writer = csv.DictWriter(fileRogue, fieldnames=fieldnames)
        writer.writerow({'Time': "BREAK", 'Latitude': datetime.today()})
        fileRogue.close()

        # C2 Variables

        # Coordinates variables
        self.roguePast = (0,0)
        self.rogueCurr = (0,0)
        self.discoPast = (0,0)
        self.discoCurr = (0,0)
        self.roguePred = (0,0)
        self.discoWayPoint = (0,0)

        self.estimated_pos_lat_disco = 0.0
        self.estimated_pos_long_disco = 0.0
        self.estimated_pos_alt_disco = 0.0
        self.estimated_pos_rel_alt_disco = 0.0
        self.estimated_pos_hdg_disco = 0.0

        self.estimated_pos_lat_rogue = 0.0
        self.estimated_pos_long_rogue = 0.0
        self.estimated_pos_alt_rogue = 0.0
        self.estimated_pos_rel_alt_rogue = 0.0
        self.estimated_pos_hdg_rogue = 0.0

        self.rogue_lat_long_tuple = []

        # Radar variables
        self.radar_id = "Not initialized"
        self.radar_rcs = 0
        self.radar_latitude = 0
        self.radar_longitude = 0
        self.radar_altitude = 0
        self.radar_stationary = True
        self.radar_category = "Not initialized"

        # Track how many cycles since the last C2 applied
        self.numC2Cycles = 0

        # Track Disco's flight status
        self.discoStatus = "STABILIZE"

        # How close should the Disco be to the rogue drone
        self.rougeDroneBoundary = 10 # Meters

        # How far should the Disco be from Rogue in altitude
        self.altitudeDiff = 5 # Meters
        
        # Within how much should the altitude be close to disco
        self.altitudeTolerance = 1 

        # Is it near rogue?
        self.distanceControl = False

        # How much altitude should disco change based on camera data
        self.camera_altitude_control = 1

        # Delays between loops
        self.coord_control_delay = 1
        self.yaw_find_delay = 10
        self.fusion_control_delay = 1

        # How much angle should the drone yaw
        self.yawControl = 10
        self.yaw_control_detailed = 5
        self.yawRate = 90

        # Speed of the disco
        self.discoSpeed = 5

        # Beginning State
        self.discoState = DiscoStates.LOCATE

        # Camera data
        self.lengthX = 0
        self.lengthY = 0
        self.widthBox = 0
        self.heightBox = 0 
        self.centerX = 0 # returns center x of the box
        self.centerY = 0 # returns center y of the box

        # Start GUI data stream
        # data_bridge.start()

        # 0 active
        # 1 coasting
        # 2 stale
        # 3 expired
        # 4 lost connection
        self.cameraStatus = server.getStatus()

        # Within what range should the detected box be in?
        self.boxToleration = 35

        # Radar detection condition
        self.rogueId = "ABC"
        self.trackTime = 0

        self.initialAltitude = 0
        self.initialLoop = True

        self.ts = []
        self.prevRadarTime = 0

        # Radar mode or GPS mode?
        self.radar_mode = int(input("0 for radar mode, 1 for GPS mode: "))
        
        if self.radar_mode == 0:
            print("Radar mode selected")
            # 2d26
            # radar.connect("10.1.220.229")
            # while not radar.connect("10.1.220.229"):
            #     time.sleep(0.1)

            # radar.connect("192.168.250.35")

            # hotspot
            while not radar.connect("10.42.0.1"):
                print("Trying to connect to radar...")
                time.sleep(0.1)
        elif self.radar_mode == 1:
            print("GPS mode selected")
        
        # Disco solo or Disco + Rogue?
        # self.solo_mode = input("Enter solo for disco solo mode. Otherwise, leave it blank: ")
        self.solo_mode = ""

        if self.solo_mode == "solo":
            print("Disco solo mode selected")
        else:
            print("Disco + Rogue mode (both drones need to be online)")
            time.sleep(1)

        pass

    # ##############################################################
    # Manage the main c3Node's main loop
    def c3node_main_loop(self):
        try:
            self.getCameraData()
            pass
            
        except:
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
        if isinstance(c3Message.message, dict):
            if list(c3Message.message.keys())[0] == 'flight_mode':
                self.discoStatus = self.triggerMgr.vars['agentMode'].msg_dict['agents']['80001']['mode']
            
            if list(c3Message.message.keys())[0] == 'agent_position':
                # Fetch GPS data in dict
                estimated_pos_dict: Dict[str, Dict[str, Dict[str, float]]] = self.triggerMgr.vars['agentPos'].estimated_position()

                if self.solo_mode == "solo":
                    # UNIX time stamp
                    timestamp_pos = time.time()

                    # Update previous disco data
                    self.discoPast = self.discoCurr

                    # Get location disco data
                    self.estimated_pos_lat_disco = estimated_pos_dict["agents"]["80001"]["lat"]
                    self.estimated_pos_long_disco = estimated_pos_dict["agents"]["80001"]["lon"]
                    self.estimated_pos_alt_disco = estimated_pos_dict["agents"]["80001"]["alt"]
                    self.estimated_pos_rel_alt_disco = estimated_pos_dict["agents"]["80001"]["relative_alt"]
                    self.estimated_pos_hdg_disco = estimated_pos_dict["agents"]["80001"]["hdg"]

                    # Record the GPS data in csv file
                    file = open(f"disco_position.csv", 'a', newline='')
                    fieldnames = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Relative Altitude', "Heading", "State"]
                    writer = csv.DictWriter(file, fieldnames=fieldnames)
                    writer.writerow({'Time': timestamp_pos, 'Latitude': self.estimated_pos_lat_disco, 'Longitude': self.estimated_pos_long_disco, 'Altitude': self.estimated_pos_alt_disco, 'Relative Altitude': self.estimated_pos_rel_alt_disco, 'Heading': self.estimated_pos_hdg_disco, 'State': self.discoState})
                    file.close()
                    
                    data_bridge.pushData((timestamp_pos, self.estimated_pos_lat_disco, self.estimated_pos_long_disco, self.estimated_pos_alt_disco, self.estimated_pos_rel_alt_disco, self.estimated_pos_hdg_disco), \
                        (0.0, 0.0, 0.0, 0.0, 0.0), self.discoStatus)

                else: 
                    if "80001" in estimated_pos_dict["agents"] and "80002" in estimated_pos_dict["agents"]:
                        # UNIX time stamp
                        timestamp_pos = time.time()

                        # Update previous disco data
                        self.discoPast = self.discoCurr

                        # Get location disco data
                        self.estimated_pos_lat_disco = estimated_pos_dict["agents"]["80001"]["lat"]
                        self.estimated_pos_long_disco = estimated_pos_dict["agents"]["80001"]["lon"]
                        self.estimated_pos_alt_disco = estimated_pos_dict["agents"]["80001"]["alt"]
                        self.estimated_pos_rel_alt_disco = estimated_pos_dict["agents"]["80001"]["relative_alt"]
                        self.estimated_pos_hdg_disco = estimated_pos_dict["agents"]["80001"]["hdg"]

                        # Record the GPS data in csv file
                        file = open(f"disco_position.csv", 'a', newline='')
                        fieldnames = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Relative Altitude', "Heading", "State"]
                        writer = csv.DictWriter(file, fieldnames=fieldnames)
                        writer.writerow({'Time': timestamp_pos, 'Latitude': self.estimated_pos_lat_disco, 'Longitude': self.estimated_pos_long_disco, 'Altitude': self.estimated_pos_alt_disco, 'Relative Altitude': self.estimated_pos_rel_alt_disco, 'Heading': self.estimated_pos_hdg_disco, 'State': self.discoState})
                        file.close()

                        self.discoCurr = (self.estimated_pos_lat_disco, self.estimated_pos_long_disco)

                        # Get location rogue data
                        self.estimated_pos_lat_rogue = estimated_pos_dict["agents"]["80002"]["lat"]
                        self.estimated_pos_long_rogue = estimated_pos_dict["agents"]["80002"]["lon"]
                        self.estimated_pos_alt_rogue = estimated_pos_dict["agents"]["80002"]["alt"]
                        self.estimated_pos_rel_alt_rogue = estimated_pos_dict["agents"]["80002"]["relative_alt"]
                        self.estimated_pos_hdg_rogue = estimated_pos_dict["agents"]["80002"]["hdg"]

                        file = open(f"rogue_position.csv", 'a', newline='')
                        fieldnames = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Relative Altitude', "Heading"]
                        writer = csv.DictWriter(file, fieldnames=fieldnames)
                        writer.writerow({'Time': timestamp_pos, 'Latitude': self.estimated_pos_lat_rogue, 'Longitude': self.estimated_pos_long_rogue, 'Altitude': self.estimated_pos_alt_rogue, 'Relative Altitude': self.estimated_pos_rel_alt_rogue, 'Heading': self.estimated_pos_hdg_rogue})
                        file.close()
                        
                        data_bridge.pushData((timestamp_pos, self.estimated_pos_lat_disco, self.estimated_pos_long_disco, self.estimated_pos_alt_disco, self.estimated_pos_rel_alt_disco, self.estimated_pos_hdg_disco), \
                            (self.estimated_pos_lat_rogue, self.estimated_pos_long_rogue, self.estimated_pos_alt_rogue, self.estimated_pos_rel_alt_rogue, self.estimated_pos_hdg_rogue), \
                            self.discoStatus)

                        rogueTrack = radar.getById(self.rogueId)

                        if self.radar_mode == 0:
                            if rogueTrack is not None and not(math.isnan(rogueTrack.lla[2])):
                                self.ts.append(time.time()-self.prevRadarTime)
                                self.prevRadarTime = time.time()
                                # Update previous rogue data
                                self.roguePast = self.rogueCurr
                                self.rogueCurr = (rogueTrack.lla[0], rogueTrack.lla[1])
                                self.rogue_lat_long_tuple.append(self.rogueCurr)
                            else:
                                if self.radar_mode == 0:
                                    self.discoState = DiscoStates.SCAN
                                    print("Radar mode activted")
                        
                        elif self.radar_mode == 1:
                            # Update previous rogue data
                            self.roguePast = self.rogueCurr
                            self.rogueCurr = (self.estimated_pos_lat_rogue, self.estimated_pos_long_rogue)
                            self.rogue_lat_long_tuple.append(self.rogueCurr)
                            

                        # Control after collecting the most recent data
                        self.checkEmergency()

                        # print(self.discoState)
                        # print(self.cameraStatus)

                        if self.discoState == DiscoStates.EMERGENCY:
                            pass

                        elif self.discoState == DiscoStates.SCAN:
                            contTracks = radar.filterTracks(self.trackTime)

                            maxrcs = 0
                            conf_track = None
                            for track in contTracks:
                                if track.rcs > maxrcs:
                                    maxrcs = track.rcs
                                    conf_track = track

                            if conf_track is not None:
                                if conf_track.category == "dronerotor":
                                    self.rogueId = conf_track.id
                                    print(self.rogueId)
                                    self.discoState = DiscoStates.LOCATE

                        elif self.discoState == DiscoStates.LOCATE:
                            if self.radar_mode == 0:
                                self.setSpeed("80001",self.discoSpeed)
                                rogueTrack = radar.getById(self.rogueId)
                                print(rogueTrack.lla)
                                if rogueTrack is not None:
                                    if self.estimated_pos_rel_alt_disco < 5:
                                        if (rogueTrack.lla[2]-self.initialAltitude) > 10:
                                            self.discoState = DiscoStates.LAUNCH
                                    else:
                                        self.discoState = DiscoStates.ALTITUDE_CONTROL
                                else:
                                    self.discoState = DiscoStates.SCAN
                            elif self.radar_mode == 1:
                                self.setSpeed("80001",self.discoSpeed)
                                
                                if self.estimated_pos_rel_alt_disco < 5:
                                    if (self.estimated_pos_rel_alt_rogue-self.initialAltitude) > 10:
                                        self.discoState = DiscoStates.LAUNCH
                                else:
                                    self.discoState = DiscoStates.ALTITUDE_CONTROL

                        elif self.discoState == DiscoStates.LAUNCH:
                            self.launch("80001", 5)
                            time.sleep(10)
                            self.discoState = DiscoStates.ALTITUDE_CONTROL

                        elif self.discoState == DiscoStates.ALTITUDE_CONTROL:
                            if self.radar_mode == 0:
                                rogueTrack = radar.getById(self.rogueId)
                                
                                if rogueTrack is not None and not(math.isnan(rogueTrack.lla[2])):
                                    print(rogueTrack.lla[2])
                                    # Set the goal altitude
                                    self.goal_altitude = rogueTrack.lla[2]-self.altitudeDiff

                                    if self.goal_altitude > 5:
                                        if self.estimated_pos_rel_alt_disco < self.goal_altitude-self.altitudeTolerance or self.estimated_pos_rel_alt_disco > self.goal_altitude+self.altitudeTolerance:
                                            self.goto_altitude("80001", self.goal_altitude)
                                            print("Goal Altitude: ", self.goal_altitude)
                                    else:
                                        print("Discovery Drone Altitude: ", self.goal_altitude)
                                        
                                    if self.estimated_pos_rel_alt_disco > self.goal_altitude-self.altitudeTolerance and self.estimated_pos_rel_alt_disco < self.goal_altitude+self.altitudeTolerance:
                                        self.prevRadarTime = time.time()
                                        self.discoState = DiscoStates.COORD_CONTROL
                                else:
                                    self.discoState = DiscoStates.SCAN
                            
                            if self.radar_mode == 1:
                                self.goal_altitude = self.estimated_pos_rel_alt_rogue-self.altitudeDiff
                                
                                if self.goal_altitude > 5:
                                    if self.estimated_pos_rel_alt_disco < self.goal_altitude-self.altitudeTolerance or self.estimated_pos_rel_alt_disco > self.goal_altitude+self.altitudeTolerance:
                                        self.goto_altitude("80001", self.goal_altitude)
                                        print("Goal Altitude: ", self.goal_altitude)
                                    else:
                                        print("Discovery Drone Altitude: ", self.goal_altitude)
                                    
                                    if self.estimated_pos_rel_alt_disco > self.goal_altitude-self.altitudeTolerance and self.estimated_pos_rel_alt_disco < self.goal_altitude+self.altitudeTolerance:
                                        self.prevRadarTime = time.time()
                                        self.discoState = DiscoStates.COORD_CONTROL


                        elif (self.discoState == DiscoStates.COORD_CONTROL) and (self.numC2Cycles % self.coord_control_delay) == 0:
                        # elif (self.discoState == DiscoStates.COORD_CONTROL):
                            # self.roguePred = self.kalmanFunction(self.rogue_lat_long_tuple)
                            self.roguePred = self.linearPrediction(self.rogue_lat_long_tuple[-2], self.rogue_lat_long_tuple[-1])
                            self.discoWayPoint = self.discoveryDroneWayPoint (self.discoCurr, self.rogue_lat_long_tuple[-1], self.roguePred)
                            print("Discovery Drone WP: ", self.discoWayPoint)
                            self.goto_location("80001", self.discoWayPoint)

                            if self.estimated_pos_rel_alt_disco < self.goal_altitude-self.altitudeTolerance or self.estimated_pos_rel_alt_disco > self.goal_altitude+self.altitudeTolerance:
                                self.discoState = DiscoStates.ALTITUDE_CONTROL
                            elif self.cameraStatus <= 1:
                                self.goto_location("80001", self.discoCurr)
                                self.discoState = DiscoStates.FUSION_CONTROL
                            elif self.distanceControl:
                                self.discoState = DiscoStates.YAW_FIND
                            

                        elif (self.discoState == DiscoStates.YAW_FIND) and (self.numC2Cycles % self.yaw_find_delay) == 0:
                            # DO YAW for 360 deg
                            self.checkDistanceFromRogue()

                            if self.estimated_pos_rel_alt_disco < self.goal_altitude-self.altitudeTolerance or self.estimated_pos_rel_alt_disco > self.goal_altitude+self.altitudeTolerance:
                                self.discoState = DiscoStates.ALTITUDE_CONTROL

                            elif self.distanceControl == False: 
                                print("Rogue moved")   
                                self.discoState = DiscoStates.COORD_CONTROL                        

                            elif self.cameraStatus <= 1:
                                self.discoState = DiscoStates.FUSION_CONTROL
                            
                            else:
                                self.yaw("80001", self.yawControl, self.yawRate)


                        elif self.discoState == DiscoStates.FUSION_CONTROL and (self.numC2Cycles % self.fusion_control_delay) == 0:
                            print("X: %d, Y: %d" % (self.centerX, self.centerY))

                            self.checkDistanceFromRogue()

                            if self.distanceControl == False: 
                                print("Rogue moved")   
                                self.discoState = DiscoStates.COORD_CONTROL
                            
                            if self.cameraStatus > 1:
                                print("Camera can't find")   
                                self.discoState = DiscoStates.YAW_FIND

                            if self.centerY < (150)-self.boxToleration:
                                self.goto_altitude("80001", self.estimated_pos_alt_disco+self.camera_altitude_control)
                            elif self.centerY > (150)+self.boxToleration:
                                self.goto_altitude("80001", self.estimated_pos_alt_disco-self.camera_altitude_control)
                            else:
                                if self.centerX < (150)-self.boxToleration:
                                    self.yaw("80001", self.yaw_control_detailed, self.yawRate)
                                elif self.centerX > (150)+self.boxToleration:
                                    self.yaw("80001", self.yaw_control_detailed, self.yawRate, "ccw")
                                else:
                                    pass


                            # Imaginary box within our toleration, alt only
                            if self.cameraStatus <= 1 and abs(self.centerY - (self.lengthY/2)) <= self.boxToleration:
                                self.discoState = DiscoStates.CAMERA_CONTROL
                            elif self.cameraStatus <= 1:
                                self.discoState = DiscoStates.FUSION_CONTROL
                            else:
                                self.discoState = DiscoStates.ALTITUDE_CONTROL
                                                
                        
                        elif self.discoState == DiscoStates.CAMERA_CONTROL:
                            self.discoState = DiscoStates.FUSION_CONTROL

                        self.numC2Cycles = self.numC2Cycles + 1
                    
                    else:
                        print("Check drone connection. Both drones are not connected to the Agent Core.")

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

        # print(f"From:{c3Message.sender} = {c3Message.message}", flush=True)
        if c3Message.sender[0:7] == 'MISSION':
            self.process_connection(c3Message.message)

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

    def checkEmergency(self):
        # if self.discoStatus == "RTL" or self.discoStatus == "LAND" or self.cameraStatus == 4:
        if self.discoStatus == "RTL" or self.discoStatus == "LAND":
            print(self.cameraStatus)
            self.discoState = DiscoStates.EMERGENCY

    def checkDistanceFromRogue(self):
        self.roguePred = self.linearPrediction(self.rogue_lat_long_tuple[-2], self.rogue_lat_long_tuple[-1])
        self.discoWayPoint = self.discoveryDroneWayPoint (self.discoCurr, self.rogue_lat_long_tuple[-1], self.roguePred)

    def getCameraData(self):
        # server.printOut() # Debugging
        self.lengthX = server.getDims()[0]
        self.lengthY = server.getDims()[1]
        self.widthBox = server.getDims()[2]
        self.heightBox = server.getDims()[3]
        self.centerX = server.getDims()[4] # returns center x of the box
        self.centerY = server.getDims()[5] # returns center y of the box

        self.cameraStatus = server.getStatus()
            
    def haversine(self, coord1, coord2):
        # Radius of the Earth in meters
        R = 6371000

        lat1, lon1 = coord1
        lat2, lon2 = coord2

        # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        # Haversine formula
        a = (math.sin(delta_lat / 2) ** 2 +
            math.cos(lat1_rad) * math.cos(lat2_rad) *
            math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Distance in meters
        distance = R * c

        return distance


    def discoveryDroneWayPoint(self, currentPos, currentRDPos, predictedDronePos):


        # print("RougeDrone: ", currentRDPos)
        # print("PredictedRougeDrone: ", predictedDronePos)
        # print("Discovery: ", currentPos)

        # Determine if RD is moving closer
        xDiff1 = currentPos[0] - predictedDronePos[0]
        yDiff1 = currentPos[1] - predictedDronePos[1]
        xDiff2 = currentPos[0] - currentRDPos[0]
        yDiff2 = currentPos[1] - currentRDPos[1]
        distance1 = ((xDiff1 ** 2) + (yDiff1 ** 2)) ** 0.5
        distance2 = ((xDiff2 ** 2) + (yDiff2 ** 2)) ** 0.5


        # Check that Discovery Drone is out of radius
        distanceBoundary = self.haversine(currentPos, currentRDPos)

        if distanceBoundary < self.rougeDroneBoundary:
            # It's close to the Rogue. Turn it over to Camera
            newWayPoint = currentPos
            self.distanceControl = True
            # print("Discovery Too Close")
            # Move Discovery Drone Away from Rouge Drone
            # rougeXInverse = -1 * currentRDPos[0]
            # rougeYInverse = -1 * currentRDPos[1]
            # Inverts Rouge Drones Coordintes, so Discovery Drone move in the opposite Direction
            # newWayPoint = (rougeXInverse, rougeYInverse)

        elif distance1 < distance2:
            self.distanceControl = False
            # Rouge Drone is moving closer to Discovery Drone
            newWayPoint = currentPos
            # print("Discovery Staying Put")

        # elif distance1 > distance2:
        else:
            # Rouge Drone is moving away from Discovery Drone
            # print("Discovery Moving")
            newWayPoint = predictedDronePos
            self.distanceControl = False
        
        return newWayPoint


    # Old Prediction Algorithm
    def linearPrediction(self, previousMeasurement, currentMeasurement):
        # Function returns a tuple of an x y coordinate 
        previousMeasurementX = previousMeasurement[0]
        previousMeasurementY = previousMeasurement[1]
        currentMeasurementX = currentMeasurement[0]
        currentMeasurementY = currentMeasurement[1]
        predictedX = (currentMeasurementX - previousMeasurementX) + currentMeasurementX
        predictedY = (currentMeasurementY - previousMeasurementY) + currentMeasurementY

        # Path to return

        coordinateToReturn = (predictedX, predictedY)

        # Vector of predicted Path

        return coordinateToReturn

    # New Prediction Algorithm
    def kalmanFunction(self, coordinateList):
        # Initialize using the first measurement's position and zero velocity
        first_measurement = coordinateList[0]
        initial_x = first_measurement[0]
        initial_y = first_measurement[1]

        kf = KalmanFilter(
        process_variance=1e-5,
        measurement_variance=1,
        initial_estimate=(initial_x, 0, initial_y, 0),  # x=0, vx=1, y=0, vy=1
        initial_uncertainty=1,
        ts=self.ts[-1]
        )


        # Kalman Filter prediction for each measurement
        predicted_positions = [(initial_x, initial_y)]

        
        future_predictions = []
        for measurement in coordinateList:
            # Predict next state (future position one step ahead)
            kf.predict()
            
            # Update with the current measurement
            kf.update(measurement)
            
            # Get the current estimated position (after update)
            current_x, _, current_y, _ = kf.get_state()
            predicted_positions.append((current_x, current_y))
            
            # Predict the future position one step ahead
            future_x, future_y = kf.predict_future(steps=1)
            future_predictions.append((future_x, future_y))

        # Plot the test measurements, noisy measurements, and Kalman Filter predictions

        originalPtR = future_predictions[-1]
        predictionToReturn = (originalPtR[0].item(), originalPtR[1].item())
        return predictionToReturn
        
    def goto_altitude(self, agent, altitude):
        msg = [f"cmd_nav_guided_goto_alt_hat({altitude})"]
        self.send_direct_message(
            agent,
            self._config['C3_ID'],
            msg)

    def goto_location(self, agent, location):
        msg = [f"cmd_nav_guided_reposition_hat({location[0]},"
               f"{location[1]})"]
        self.send_direct_message(
            agent,
            self._config['C3_ID'],
            msg)
        time.sleep(1)

    def launch(self, agent, altitude):
        msg = [f"cmd_sys_mode_change(guided)"]
        self.send_direct_message(
            agent,
            self._config['C3_ID'],
            msg)
        time.sleep(1)
        msg = [f"cmd_sys_arm_disarm(arm, delay=5)"]
        self.send_direct_message(
            agent,
            self._config['C3_ID'],
            msg)
        time.sleep(1)
        msg = [f"cmd_nav_guided_takeoff({altitude})"]
        self.send_direct_message(
            agent,
            self._config['C3_ID'],
            msg)
        time.sleep(1)
    
    def yaw(self, agent, rate, angle, direction: str="cw"):
        if direction == "ccw":
            dir = -1
        else:
            dir = 1
        msg = [f"cmd_nav_guided_set_yaw({angle}, {rate}, True, {dir})"]
        self.send_direct_message(
            agent,
            self._config['C3_ID'],
            msg)
        
    def setSpeed(self, agent, speed):
        msg = [f"cmd_nav_guided_set_speed({speed})"]
        self.send_direct_message(
            agent,
            self._config['C3_ID'],
            msg
        )

    def process_connection(self, connection_dict_str):

        connection_dict = json.loads(connection_dict_str)
        try:
            for agent, value in self.home_dist.dict.items():
                dist1 = value
                lat2 = connection_dict['lat']
                lon2 = connection_dict['lon']
                dist2 = C3NodeUtils.get_lat_lon_2_lat_lon_range(
                    self.home_location[0], self.home_location[1],
                    lat2, lon2)

                # print(f"dist1: {dist1} / dist2: {dist2}", flush=True)

                if (connection_dict['target_known'] or self.target_known):
                    # print(f"Agent: {agent} // opposing_target_known: "
                    #       f"{connection_dict['target_known']}")
                    self.target_known = True
                    if (dist1 < dist2):
                        self.goto_location(agent, self.home_location)
                    elif (dist1 > dist2):
                        self.target_known = True
                        self.goto_location(agent, self.target_location)

        except (KeyError):
            pass

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_estimate, initial_uncertainty, ts):
        # State: [x, velocity_x, y, velocity_y]
        self.state_dim = 4
        self.measurement_dim = 2  # We now measure x and y

        timeStep = ts # Time step set to 1 second

        # Process noise covariance (Q) - 4x4 matrix (x, vx, y, vy)
        self.Q = np.eye(self.state_dim) * process_variance
        
        # Measurement noise covariance (R) - 2x2 matrix (x, y)
        self.R = np.eye(self.measurement_dim) * measurement_variance
        
        # Initial state estimate
        self.x = np.array([
            [initial_estimate[0]],  # x position
            [initial_estimate[1]],  # x velocity
            [initial_estimate[2]],  # y position
            [initial_estimate[3]]   # y velocity
        ])
        
        # Initial uncertainty (covariance matrix)
        self.P = np.eye(self.state_dim) * initial_uncertainty

        # State transition matrix (F) for constant velocity model (dt=1)
        self.F = np.array([
            [1, timeStep, 0, 0],  # x updates: x + vx*dt
            [0, 1, 0, 0],  # vx remains constant
            [0, 0, 1, timeStep],  # y updates: y + vy*dt
            [0, 0, 0, 1]   # vy remains constant
        ])

        # Measurement matrix (H) - maps state to measurement [x, y]
        self.H = np.array([
            [1, 0, 0, 0],  # measure x
            [0, 0, 1, 0]   # measure y
        ])

    def predict(self):
        # Predict state and covariance
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        # Convert measurement to 2x1 vector [x, y]
        z = np.array([[measurement[0]], [measurement[1]]])
        
        # Innovation (measurement residual)
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman Gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(self.state_dim) - K @ self.H) @ self.P

    def get_state(self):
        # Return current x, y positions and velocities
        return self.x[0, 0], self.x[1, 0], self.x[2, 0], self.x[3, 0]

    def predict_future(self, steps=1):
        # Predict future state (without updating the filter's internal state)
        future_x = self.x.copy()
        future_P = self.P.copy()
        F_step = np.linalg.matrix_power(self.F, steps)  # F^steps for multi-step prediction
        future_x = F_step @ future_x
        future_P = F_step @ future_P @ F_step.T + self.Q * steps
        return future_x[0, 0], future_x[2, 0]  # Return future x and y positions


if __name__ == "__main__":

    agent_count = 1
    for agent in range(1, agent_count + 1):
        c3Mission_1Node = Agent_Mission_Node(
            config_file=f"C3Mission_1.yaml")
        c3Mission_1Node.start()
