''' Utility Functions and Methods for the C3 Nodes '''

import math  # noqa: F401
import random
import numpy as np

from classes.agentutils import AgentUtils  # noqa: F401
from classes.trigger import TriggerManager

# from geopy.distance import distance
# from geopy import Point
# import numpy as np
# from shapely.geometry import Polygon, Point as ShapelyPoint, LineString
# from shapely import is_closed

# from geopy import Point
# from shapely.geometry import Polygon, Point as ShapelyPoint, LineString
# from shapely.affinity import translate
# from geopy.distance import geodesic
# from geopy.distance import distance


class C3NodeUtils():

    # from classes.c3_node import C3NodeMessage

    @classmethod
    def list_of_distances(cls,
                          lat_lon_alt_cnv: TriggerManager.C3NodePositionVar):
        '''
        Takes the C3NodeVar of lat_lon_alt and creates a list of one-to-one
        distances that are sorted from closest to farthest
        '''

        # print(f"pos: {lat_lon_alt_cnv}", flush=True)

        try:

            if hasattr(lat_lon_alt_cnv, 'msg_dict'):
                main_dict = lat_lon_alt_cnv.msg_dict
            else:
                main_dict = lat_lon_alt_cnv

            distances = {}
            agent_list = list(main_dict['agents'].keys())

            if len(main_dict['agents'].keys()) > 1:

                for i in range(len(agent_list)):
                    for j in range(i + 1, len(agent_list)):
                        agent1 = str(agent_list[i])
                        agent2 = str(agent_list[j])
                        pair_key = f"{agent1}__{agent2}"
                        lat1 = main_dict['agents'][agent1]['lat']
                        lon1 = main_dict['agents'][agent1]['lon']
                        lat2 = main_dict['agents'][agent2]['lat']
                        lon2 = main_dict['agents'][agent2]['lon']
                        alt1 = main_dict['agents'][agent1]['alt']
                        alt2 = main_dict['agents'][agent2]['alt']
                        dist = AgentUtils.lat_lon_distance_light(
                            lat1, lon1, lat2, lon2, alt1, alt2
                        )
                        distances[pair_key] = dist

                sorted_distances = dict(sorted(distances.items(),
                                               key=lambda item: item[1]))
                return sorted_distances

            else:
                return None

        except (AttributeError, TypeError):
            pass

    # @classmethod
    # def calculate_time_to_intersection(cls,
    #                                    agent_pos,
    #                                    boundary_points):

    #     def calculate_closest_bearing(vehicle_location,
    #                                   boundary_points):

    #         vehicle_point = ShapelyPoint(vehicle_location)
    #         # boundary_polygon = Polygon(boundary_points)

    #         # Ensure the polygon is valid and closed
    #         # if not boundary_polygon.is_valid or not boundary_polygon.is_closed:
    #         #     raise ValueError("The boundary points do not form a valid polygon")

    #         min_distance = float('inf')
    #         closest_point = None

    #         # Iterate through each edge of the polygon
    #         for i in range(len(boundary_points)):
    #             p1 = boundary_points[i]
    #             p2 = boundary_points[(i + 1) % len(boundary_points)]
    #             line = LineString([p1, p2])
    #             distance_to_line = vehicle_point.distance(line)

    #             if distance_to_line < min_distance:
    #                 min_distance = distance_to_line
    #                 closest_point = line.interpolate(line.project(vehicle_point))

    #         # Calculate the range (distance) using geopy
    #         vehicle_geo_point = Point(vehicle_location[0], vehicle_location[1])
    #         closest_geo_point = Point(closest_point.x, closest_point.y)
    #         range_to_edge = distance(vehicle_geo_point, closest_geo_point).meters

    #         # Calculate the bearing
    #         def calculate_initial_compass_bearing(pointA, pointB):
    #             lat1 = np.radians(pointA.latitude)
    #             lat2 = np.radians(pointB.latitude)
    #             diffLong = np.radians(pointB.longitude - pointA.longitude)
    #             x = np.sin(diffLong) * np.cos(lat2)
    #             y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(diffLong))
    #             initial_bearing = np.arctan2(x, y)
    #             initial_bearing = np.degrees(initial_bearing)
    #             compass_bearing = (initial_bearing + 360) % 360
    #             return compass_bearing

    #         bearing_to_edge = calculate_initial_compass_bearing(vehicle_geo_point, closest_geo_point)

    #         return range_to_edge, bearing_to_edge

    #     def create_valid_polygon(points):
    #         # # Remove consecutive duplicate points
    #         # cleaned_points = []
    #         # for i, point in enumerate(points):
    #         #     if i == 0 or point != points[i - 1]:
    #         #         cleaned_points.append(point)

    #         # # Ensure the polygon is closed
    #         # if cleaned_points[0] != cleaned_points[-1]:
    #         #     cleaned_points.append(cleaned_points[0])

    #         # polygon = Polygon(cleaned_points)

    #         # if not polygon.is_valid:
    #         #     raise ValueError("The boundary points do not form a valid polygon")

    #         polygon = Polygon(points)

    #         return polygon

    #     def find_intersection_point(vehicle_location,
    #                                 velocity_vector,
    #                                 boundary_polygon):

    #         vehicle_point = ShapelyPoint(vehicle_location)

    #         # Create a line representing the vehicle's trajectory
    #         trajectory_line = LineString(
    #             [vehicle_point,
    #              translate(
    #                  vehicle_point,
    #                  xoff=velocity_vector[0]*1e5,
    #                  yoff=velocity_vector[1]*1e5)])

    #         min_distance = float('inf')
    #         closest_intersection = None

    #         # Check intersection with each segment of the polygon boundary
    #         for i in range(len(boundary_polygon.exterior.coords) - 1):
    #             boundary_segment = LineString(
    #                 [boundary_polygon.exterior.coords[i],
    #                  boundary_polygon.exterior.coords[i + 1]])
    #             intersection = trajectory_line.intersection(
    #                 boundary_segment)

    #             if not intersection.is_empty:
    #                 distance_to_intersection = vehicle_point.distance(intersection)
    #                 if distance_to_intersection < min_distance:
    #                     min_distance = distance_to_intersection
    #                     closest_intersection = intersection

    #         # if closest_intersection is None:
    #         #     raise ValueError("No intersection found between the vehicle's trajectory and the polygon boundary")

    #         return closest_intersection

    #     # ###################### Main ##########################
    #     agent_list = list(agent_pos.msg_dict['agents'].keys())
    #     boundary_polygon = create_valid_polygon(boundary_points)
        
    #     if len(agent_pos.msg_dict['agents'].keys()) > 0:

    #         # for i in range(len(agent_list)):
    #         #     agent = agent_pos.msg_dict['agents'][agent_list[i]]
    #         #     print(f"agent: {i} - {agent}")

    #         time_to_intersect = {}
    #         for i in range(len(agent_list)):

    #             agent = agent_pos.msg_dict['agents'][agent_list[i]]
    #             vehicle_location = (agent['lat'], agent['lon'])
    #             velocity_vector = (agent['vx']/100.0, agent['vy']/100.0)
    #             vehicle_speed = math.sqrt(velocity_vector[0]**2 +
    #                                       velocity_vector[1]**2)

    #             # print(f"agent: {i+1} - {agent['lon']}")

    #             intersection_point = find_intersection_point(
    #                 vehicle_location, velocity_vector, boundary_polygon)

    #             # print(f"agent: {i+1} - {agent['lon']}")

    #             r, b = calculate_closest_bearing(
    #                 vehicle_location, boundary_points)

    #             # print(f"r: {r}, b: {b}")

    #             if intersection_point is None:

    #                 distance_to_intersection = None
    #                 time_to_intersection = None
    #                 intersection_point = None

    #             else:

    #                 vehicle_geo_point = (
    #                     vehicle_location[0], vehicle_location[1])
    #                 intersection_geo_point = (
    #                     intersection_point.x, intersection_point.y)

    #                 distance_to_intersection = geodesic(
    #                     vehicle_geo_point, intersection_geo_point).meters

    #                 speed = np.sqrt(
    #                     velocity_vector[0]**2 + velocity_vector[1]**2)  # m/s
    #                 time_to_intersection = (distance_to_intersection
    #                                         / speed)  # sec

    #             # print(f"time-to-edge: {time_to_intersection}")
    #             time_to_intersect[agent_list[i]] = {
    #                 'position': vehicle_location,
    #                 'distance': distance_to_intersection,
    #                 'vehicle_vel': velocity_vector,
    #                 'speed': vehicle_speed,
    #                 'time_to_intersect': time_to_intersection,
    #                 'intersection_point': intersection_point,
    #                 'range_to_closest': r,
    #                 'bearing_to_closest': b,
    #             }

    #         return time_to_intersect

    @classmethod
    def get_recip_angle(cls,
                        angle: float):

        def reduce_angle(angle):
            return angle % 360

        new_angle = reduce_angle(angle + 180)
        return new_angle
        # if band is not None:
        #     low_angle = reduce_angle(new_angle - band/2)
        #     high_angle = reduce_angle(new_angle + band/2)
        # else:
        #     low_angle = new_angle
        #     high_angle = new_angle

        # return low_angle, high_angle

    @classmethod
    def get_random_angle_from_band(cls, band: list):

        '''
        It's a clockwise angle from band[0] to band[1]
        '''

        def reduce_angle(angle):
            return angle % 360

        band_diff = band[1] - band[0]
        if band_diff >= 0:
            rand_val = random.uniform(0, band_diff)
        else:
            pos_band_diff = 360 + band_diff
            rand_val = random.uniform(0, pos_band_diff)

        random_angle = reduce_angle(band[0] + rand_val)

        return random_angle

    @classmethod
    def get_random_angle_from_angle_band(cls, angle, band):

        def reduce_angle(angle):
            return angle % 360

        start_band = angle - band/2
        stop_band = angle + band/2

        new_angle = cls.get_random_angle_from_band(
            [start_band, stop_band])

        return new_angle

    @classmethod
    def hit_boundary(cls, agent_pos_dict, boundary_points):

        def convert_velocity_to_degrees(lat, velocity_cm_sec):
            """
            Converts velocity from cm/sec to degrees/sec.
            Parameters:
                lat: Latitude of the position in degrees.
                velocity_cm_sec: Velocity vector in cm/sec (x_velocity, y_velocity).
            Returns:
                Velocity vector in degrees/sec (delta_lat, delta_lon).
            """
            meters_per_degree_lat = 111139
            meters_per_degree_lon = 111139 * np.cos(np.radians(lat))

            velocity_m_sec = velocity_cm_sec / 100  # Convert to meters/sec

            delta_lat = velocity_m_sec[0] / meters_per_degree_lat
            delta_lon = velocity_m_sec[1] / meters_per_degree_lon

            return np.array([delta_lat, delta_lon])

        def line_intersection(p1, p2, q1, q2):
            """
            Given two line segments p1->p2 and q1->q2, determine if they intersect.
            If they do, return the intersection point and time to intercept.
            """

            # Define vectors
            p = np.array(p1)
            r = np.array(p2) - p
            q = np.array(q1)
            s = np.array(q2) - q

            # Solve for t and u (parametric solution)
            r_cross_s = np.cross(r, s)
            q_minus_p = q - p
            qmp_cross_r = np.cross(q_minus_p, r)

            if r_cross_s == 0 and qmp_cross_r == 0:
                # Collinear
                return None
            elif r_cross_s == 0 and qmp_cross_r != 0:
                # Parallel, non-intersecting
                return None
            else:
                # Intersection point calculation
                t = np.cross(q_minus_p, s) / r_cross_s
                u = qmp_cross_r / r_cross_s
                # print(f"p1: {p1} / p2: {p2}, / q1: {q1} / q2: {q2} -- "
                #       f"t: {t} / u: {u} / {p + t * r}")

                if t > 0 and 0 <= u <= 1:
                    intersection_point = p + t * r
                    # Intersection point and time to intercept
                    # print(f"Intersect:{p1} -> {intersection_point} / t: {t}")
                    return p1, intersection_point, t, (q1, q2)
                else:
                    # No intersection
                    # print("No Intersect")
                    return None

        def find_intersections(position, velocity_cm_sec, boundary_points):
            """
            Find all intersections of the vehicle's path with
            the boundary segments.
            Parameters:
                position: Current position of the vehicle (lat, lon).
                velocity: Velocity vector (delta_lat, delta_lon).
            Returns:
                A list of tuples (intersection_point, time_to_intercept),
                sorted by time to intercept.
            """
            velocity_deg_sec = convert_velocity_to_degrees(
                position[0], velocity_cm_sec)

            intersections = []

            # print("---------")
            for i in range(len(boundary_points) - 1):
                segment_start = boundary_points[i]
                segment_end = boundary_points[i + 1]

                # print(f"segment: ({segment_start}, {segment_end})")
                result = line_intersection(position,
                                           position + velocity_deg_sec,
                                           segment_start,
                                           segment_end)

                if result is not None:
                    intersections.append(result)

            # print(f"intersections: {intersections}")
            # Sort intersections by time to intercept
            intersections.sort(key=lambda x: x[1])
            return intersections

        def is_point_inside_polygon(point, boundary_points):
            """
            Determine if the point is inside a closed polygon using the ray-casting algorithm.
            Parameters:
                point: The point to check (lat, lon).
                boundary_points: List of points defining the closed polygon boundary (lat, lon).
            Returns:
                True if the point is inside the polygon, False if outside.
            """
            x, y = point
            n = len(boundary_points)
            inside = False

            p1x, p1y = boundary_points[0]
            for i in range(1, n):
                p2x, p2y = boundary_points[i % n]

                if y > min(p1y, p2y):
                    if y <= max(p1y, p2y):
                        if x <= max(p1x, p2x):
                            if p1y != p2y:
                                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
                p1x, p1y = p2x, p2y

            return inside

        # ############ main ###############
        # print('-----------')
        # print(f"AGENT_DICT: {agent_pos_dict.msg_dict}")
        # print(f"AGENTS: {agent_pos_dict.msg_dict['agents'].keys()}")

        if hasattr(agent_pos_dict, 'msg_dict'):
            main_dict = agent_pos_dict.msg_dict
        else:
            main_dict = agent_pos_dict

        agent_intersections = {}
        for agent, value in main_dict['agents'].items():
            intersections = None
            # print(f"AGENT: {agent} / VALUE: {value}")
            try:
                current_position = np.array([
                    float(value['lat']),
                    float(value['lon'])
                    ])
                velocity_vector = np.array([
                    float(value['vx']),
                    float(value['vy'])
                    ])

                if is_point_inside_polygon(
                        (float(value['lat']),
                         float(value['lon'])),
                        boundary_points
                        ):
                    intersections = find_intersections(current_position,
                                                       velocity_vector,
                                                       boundary_points)

                if intersections is not None:
                    # print(f"intersections: {intersections}")

                    position = (intersections[0][0][0],
                                intersections[0][0][1])
                    intersecting_segment = (intersections[0][3])

                    ortho_heading = cls.get_orthogonal_heading_to_line(
                        position,
                        intersecting_segment[0],
                        intersecting_segment[1]
                    )

                    agent_intersections[agent] = {
                        "position": position,
                        "intersection": (
                            intersections[0][1][0],
                            intersections[0][1][1]
                        ),
                        "time_to_intersect": intersections[0][2],
                        "intersecting_segment": intersections[0][3],
                        "ortho_heading": ortho_heading
                    }

            except (KeyError, IndexError):
                # print("KeyError or IndexError")
                pass

        if agent_intersections != {}:
            # print(f"Agent Int: {agent_intersections}")
            return agent_intersections
        else:
            # print("I am False")
            return

    @classmethod
    def get_orthogonal_heading_to_line(cls,
                                       vehicle_pos,
                                       line_start,
                                       line_end):

        def latlon_to_cartesian(lat, lon):
            lat, lon = np.radians(lat), np.radians(lon)
            x = np.cos(lat) * np.cos(lon)
            y = np.cos(lat) * np.sin(lon)
            z = np.sin(lat)
            return np.array([x, y, z])

        def cartesian_to_latlon(cartesian):
            x, y, z = cartesian
            lon = np.arctan2(y, x)
            hyp = np.sqrt(x * x + y * y)
            lat = np.arctan2(z, hyp)
            return np.degrees(lat), np.degrees(lon)

        def orthogonal_intercept(vehicle_pos, line_start, line_end):
            P = latlon_to_cartesian(*vehicle_pos)
            A = latlon_to_cartesian(*line_start)
            B = latlon_to_cartesian(*line_end)
            
            AB = B - A
            AP = P - A
            
            # Projection formula to find the closest point on the line segment
            t = np.dot(AP, AB) / np.dot(AB, AB)
            
            # Clamp t to the segment [0, 1]
            t = max(0, min(1, t))
            
            C = A + t * AB
            intercept_latlon = cartesian_to_latlon(C)
            return intercept_latlon

        def calculate_bearing(lat1, lon1, lat2, lon2):
            lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
            
            dlon = lon2 - lon1
            x = np.sin(dlon) * np.cos(lat2)
            y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
            initial_bearing = np.arctan2(x, y)
            
            # Convert bearing from radians to degrees and normalize to [0, 360)
            bearing = (np.degrees(initial_bearing) + 360) % 360
            return bearing

        intercept_point = orthogonal_intercept(
            vehicle_pos, line_start, line_end)
        heading = calculate_bearing(
            vehicle_pos[0], vehicle_pos[1],
            intercept_point[0], intercept_point[1])

        # print(f"Heading: {heading}")

        return heading

    @classmethod
    def heading_to_centroid(cls, position, boundary_points):

        def lat_lon_to_cartesian(lat, lon):
            lat, lon = np.radians(lat), np.radians(lon)
            x = np.cos(lat) * np.cos(lon)
            y = np.cos(lat) * np.sin(lon)
            z = np.sin(lat)
            return x, y, z

        def cartesian_to_lat_lon(x, y, z):
            lon = np.arctan2(y, x)
            hyp = np.sqrt(x * x + y * y)
            lat = np.arctan2(z, hyp)
            return np.degrees(lat), np.degrees(lon)

        def centroid_of_lat_lons(lat_lons):
            x, y, z = 0, 0, 0
            for lat, lon in lat_lons:
                cx, cy, cz = lat_lon_to_cartesian(lat, lon)
                x += cx
                y += cy
                z += cz
            x /= len(lat_lons)
            y /= len(lat_lons)
            z /= len(lat_lons)
            return cartesian_to_lat_lon(x, y, z)

        # Example usage
        # lat_lons = [(39.7392, -104.9903), (34.0522, -118.2437), (40.7128, -74.0060)]
        centroid = centroid_of_lat_lons(boundary_points)
        print("Centroid:", centroid)

    @classmethod
    def check_heading_within_bounds(cls, position, heading, boundary_points):

        def calculate_destination_point(start_lat,
                                        start_lon,
                                        bearing,
                                        distance_km):

            # Calculate the endpoint using geopy given a start point,
            # bearing, and distance.
            start_point = geodesic(kilometers=distance_km).\
                destination((start_lat, start_lon), bearing)
            return start_point.latitude, start_point.longitude

        def is_heading_pointing_to_polygon(start_lat,
                                           start_lon,
                                           bearing,
                                           polygon_coords,
                                           distance_km=10):
            # Define the polygon
            polygon = Polygon(polygon_coords)

            # Calculate the destination point based on heading and distance
            end_lat, end_lon = calculate_destination_point(start_lat,
                                                           start_lon,
                                                           bearing,
                                                           distance_km)
            print(f"end_point: {end_lat}, {end_lon}")

            # Create a line from start point to the calculated endpoint
            line = LineString([(start_lon, start_lat), (end_lon, end_lat)])

            # Check if the line intersects with the polygon
            return line.intersects(polygon)

        # polygon_coords = [(lat1, lon1), (lat2, lon2), (lat3, lon3), ...]
        # # List of lat/lon tuples
        # start_lat, start_lon = position[0], position[1]
        # # Example start point

        print(f"{position[0]} / {position[1]} / {heading}")
        result = is_heading_pointing_to_polygon(position[0],
                                                position[1],
                                                heading,
                                                boundary_points)

        result = cls.hit_boundary()

        print("Intersects:", result)
        return result

    @classmethod
    def get_lat_lon_2_lat_lon_range(cls, lat1, lon1, lat2, lon2):

        # Use the Haversine formula
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(
            math.radians, [lat1, lon1, lat2, lon2])

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * \
            math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.asin(math.sqrt(a))

        # Radius of Earth in kilometers. Use 3956 for miles
        r = 6371

        # Calculate the result
        return c * r * 1000

    @classmethod
    def get_range_to_lat_lon(cls, agent_pos_dict, target):

        # def haversine(lat1, lon1, lat2, lon2):
        #     # Convert latitude and longitude from degrees to radians
        #     lat1, lon1, lat2, lon2 = map(
        #         math.radians, [lat1, lon1, lat2, lon2])

        #     # Haversine formula
        #     dlat = lat2 - lat1
        #     dlon = lon2 - lon1
        #     a = math.sin(dlat / 2)**2 + math.cos(lat1) * \
        #         math.cos(lat2) * math.sin(dlon / 2)**2
        #     c = 2 * math.asin(math.sqrt(a))

        #     # Radius of Earth in kilometers. Use 3956 for miles
        #     r = 6371

        #     # Calculate the result
        #     return c * r * 1000

        if hasattr(agent_pos_dict, 'msg_dict'):
            main_dict = agent_pos_dict.msg_dict
        else:
            main_dict = agent_pos_dict

        agent_ranges = {}
        if main_dict is not None:
            for agent, value in main_dict['agents'].items():

                try:

                    range = cls.get_lat_lon_2_lat_lon_range(
                        float(value['lat']),
                        float(value['lon']),
                        float(target[0]),
                        float(target[1]))
                    agent_ranges[agent] = range

                except (KeyError, IndexError):
                    pass

        return agent_ranges
