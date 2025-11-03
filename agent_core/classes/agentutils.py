''' Utility Functions and Methods for the Agent '''

from dataclasses import dataclass
import os
import sys
import yaml
import json
import math
import time
import threading
from functools import partial


@dataclass
class Position:

    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0


class TimeKeeper:
    def __init__(self, interval, action):
        self.interval = interval
        self.action = action
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

    def run(self):
        while True:
            time.sleep(self.interval)
            self.action()


class AgentUtils:

    def __init__(self):

        self.config = self.get_config_dict()

    @classmethod
    def get_config_dict(cls, yaml_file):
        ''' Gets the config file for this agent '''
        # Load config yaml file and store it in config dictionary
        here = os.path.dirname(os.path.abspath(__file__))
        # here = here[:here.find('/classes')]
        here = os.path.normpath(here)
        here = os.path.dirname(here)
        # print(here)
        with open(os.path.join(here, yaml_file), "r") as f:
            config = yaml.safe_load(f)

        if '-sa' in sys.argv:
            config['SITL_ADDRESS'] = sys.argv[sys.argv.index('-ma') + 1]
        if '-aid' in sys.argv:
            config['AGENT_ID'] = sys.argv[sys.argv.index('-aid') + 1]
        if '-sid' in sys.argv:
            config['SYS_ID'] = int(sys.argv[sys.argv.index('-sid') + 1])
        if '-asp' in sys.argv:
            config['AGENT_SERVER_PORT'] = int(
                sys.argv[sys.argv.index('-asp') + 1])

        return config

    @classmethod
    def return_json(cls, obj):
        ''' Returns the JSON string of the Agent_Status class variables 
            This is used to send the agent as a JSON over zmq '''

        final_attribute_list = [a for a in dir(obj) if not a.startswith('__')
                                and not callable(getattr(obj, a))]
        final_attribute_dict = dict()
        for attribute in final_attribute_list:
            try:
                json.dumps(getattr(obj, attribute))
                attribute_value = getattr(obj, attribute)
            except:
                attribute_value = getattr(obj, attribute).__str__()
            final_attribute_dict.update({attribute: attribute_value})

        final_attribute_json = json.dumps(final_attribute_dict)
        return final_attribute_json

    @classmethod
    def lat_lon_distance_heavy(cls, lat1, lon1, lat2, lon2,
                               alt1=None, alt2=None):
        """
        Calculate the great circle distance between two points
        on the earth specified in decimal degrees.
        """
        # Convert latitude and longitude from decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * \
            math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Radius of Earth in meters.

        if ((alt1 is not None) and (alt2 is not None)):
            distance = math.sqrt(distance**2 + (abs(alt1-alt2)**2))

        return distance

    @classmethod
    def lat_lon_distance_light(cls, lat1, lon1, lat2, lon2,
                               alt1=None, alt2=None):
        """
        Equirectangular approximation to calculate the distance between
        two points on the earth specified in decimal degrees.
        """
        # Convert latitude and longitude from decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Approximation formula
        x = (lon2 - lon1) * math.cos(0.5 * (lat2 + lat1))
        y = lat2 - lat1
        distance = math.sqrt(x * x + y * y) * 6371000  # Radius of Earth in m

        if ((alt1 is not None) and (alt2 is not None)):
            distance = math.sqrt(distance**2 + (abs(alt1-alt2)**2))

        return distance

    @classmethod
    def calculate_destination_point_heavy(cls, lat, lon, bearing, distance):
        """
        Calculate the destination point given starting point,
        bearing, and distance.
        """
        # Earth radius in meters
        R = 6371000.0

        # Convert latitude and longitude from decimal degrees to radians
        lat = math.radians(lat)
        lon = math.radians(lon)
        bearing = math.radians(bearing)

        # Calculate destination point
        lat_destination = math.asin(
            math.sin(lat) * math.cos(distance / R) +
            math.cos(lat) * math.sin(distance / R) * math.cos(bearing))
        lon_destination = lon + math.atan2(
            math.sin(bearing) * math.sin(distance / R) * math.cos(lat),
            math.cos(distance / R) - math.sin(lat) * math.sin(lat_destination))

        # Convert latitude and longitude from radians to decimal degrees
        lat_destination = math.degrees(lat_destination)
        lon_destination = math.degrees(lon_destination)

        return lat_destination, lon_destination

    @classmethod
    def calculate_destination_point_light(cls, lat, lon, bearing, distance):
        """
        Calculate the destination point given starting point,
        bearing, and distance using the spherical law of cosines formula.
        """
        # Convert latitude and longitude from decimal degrees to radians

        lat = math.radians(lat)
        lon = math.radians(lon)
        bearing = math.radians(bearing)

        # Calculate destination point
        destination_lat = math.asin(math.sin(lat) *
                                    math.cos(distance / 6371000) +
                                    math.cos(lat) *
                                    math.sin(distance / 6371000) *
                                    math.cos(bearing))
        destination_lon = lon + math.atan2(
            math.sin(bearing) *
            math.sin(distance / 6371000) *
            math.cos(lat), math.cos(distance / 6371000) - math.sin(lat) *
            math.sin(destination_lat))

        # Convert latitude and longitude fr om radians to decimal degrees
        destination_lat = math.degrees(destination_lat)
        destination_lon = math.degrees(destination_lon)

        return destination_lat, destination_lon
