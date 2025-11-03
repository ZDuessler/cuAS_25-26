import pygame, sys
from pygame.locals import *
import math
import sympy as sp

from Kalman_sim_csv_converter import csvConverter

from Kalman_class import KalmanFilter



def discoveryDroneWayPoint(currentPos, currentRDPos, predictedDronePos):

    closer = False

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

    # print("Dist1: ", distance1)
    # print("Dist2: ", distance2)
    if distance1 < distance2:
        closer = False
        print("need to stay put")
    # elif distance1 - 50 < distance2:
    #     closer = False

    if distance1 > distance2:
        closer = True
        print("need to get closer")


    # RD moving away from DD
    if closer == True:
        a = -1 * yDiff1 / xDiff1

        # Define the variables
        x, y = sp.symbols('x y')


        # Define the equations
        eq1 = sp.Eq(a*x + y, 0)
        eq2 = sp.Eq(x**2 + y**2, 100)

        # Solve the system of equations
        solution = sp.solve((eq1, eq2), (x, y))

        #print("Solution: ", solution)

        xCalc = solution[0][0]
        yCalc = solution[0][1]
        newX = currentPos[0] + xCalc
        newY = currentPos[1] + yCalc
        newWayPoint = (round(newX), round(newY))

    # RD moving closer to DD
    else:
        newWayPoint = currentPos
        #print("Here")


    #newWayPoint = (650,700)
    #print(newWayPoint)

    return newWayPoint

def linearPrediction(previousMeasurement, currentMeasurement):
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

# Function to generate a curved path
def generate_curved_path(start, control, end, step=10):
    path = []
    
    # Calculate the distance between points
    num_steps = int(math.hypot(end[0] - start[0], end[1] - start[1]) / step)

    for i in range(num_steps + 1):
        t = i / num_steps
        x = (1 - t) ** 2 * start[0] + 2 * (1 - t) * t * control[0] + t ** 2 * end[0]
        y = (1 - t) ** 2 * start[1] + 2 * (1 - t) * t * control[1] + t ** 2 * end[1]
        path.append((int(x), int(y)))

    return path

def generate_straight_path(start, end):

    # Make the rouge drone path
    #rougeDronePath = [(100, 700), (100, 600), (100, 500), (100, 400), (100, 300), (100, 200)]
    # Starting and ending coordinates
    

    # Generate new path with 10-unit increments
    rougeDronePathStraight = []

    # Create points between the start and end
    for y in range(start[1], end[1] - 1, -10):  # Use -10 to decrement
        rougeDronePathStraight.append((start[0], y))

    # Include the end point
    rougeDronePathStraight.append(end)

    return rougeDronePathStraight


def determinePath(desiredPath, filename, drone):

    start = (100, 700)
    end = (100, 200)

    startDD = (700, 700)
    endDD = (700, 200)

    rougeDronePathStraight = generate_straight_path(start, end)
    discoveryDronePathStraight = generate_straight_path(startDD, endDD)

    

    ####################### Rogue Drone Curved Path #####################################
    mid = (400, 300)
    end_curve = (600, 500)
    rougeDronePathCurved = generate_curved_path(start, mid, end_curve)


    ######################### Rouge Drone CSV Path ######################################
    print("Converting to coords")
    rougeDronePathCSV = csvConverter(filename)
    if drone == "R":
        if desiredPath == "S":
            pathToReturn = rougeDronePathStraight
        elif desiredPath == "C":
            pathToReturn = rougeDronePathCurved
        elif desiredPath == "V":
            pathToReturn = rougeDronePathCSV
    else:
        pathToReturn = discoveryDronePathStraight
    return pathToReturn

def Kalman_coordinate(previousEstimate, currentMeasurement):
    A = 1
    B = 1
    C = 1
    Q = 1
    R = 1
    kf = KalmanFilter(A, B, C, Q, R)

    print(kf)