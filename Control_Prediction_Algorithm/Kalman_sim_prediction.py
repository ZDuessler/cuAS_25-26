import pygame, sys
from pygame.locals import *
import math
import numpy as np


def linearPredictionVector(previousMeasurement, currentMeasurement):
    # Starting point
    start = np.array([previousMeasurement[0], previousMeasurement[1]])

    # Direction vector (example: from start to end point)
    direction = np.array([currentMeasurement[0], currentMeasurement[1]]) - start  # This defines the direction

    # Normalize the direction vector
    direction_normalized = direction / np.linalg.norm(direction)

    # Define the length of the vector
    length = 500  # Change this to whatever length you want

    # Create the new endpoint
    end = start + direction_normalized * length

    return (start, end)


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
