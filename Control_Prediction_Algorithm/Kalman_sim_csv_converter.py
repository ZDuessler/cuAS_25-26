from pygame.locals import *
import pandas as pd


# Csv converter function
def csvConverter(filename):
    # Read in the values for a csv

    data = pd.read_csv(filename, encoding='ISO-8859-1')
    #print(data)

    # Extract Latitude and Longitude values into lists
    x_axis = data['Latitude'].tolist()
    y_axis = data['Longitude'].tolist()

    # Print the results
    # print("X Axis (Latitude):", x_axis)
    # print("Y Axis (Longitude):", y_axis)

    # Convert to the simulation display coordinates
    new_x = []
    sim_min = 100
    sim_max = 700
    x_min = min(x_axis)
    x_max = max(x_axis)

    new_y = []
    y_min = min(y_axis)
    y_max = max(y_axis)

    for i, j in zip(x_axis, y_axis):
        new_x_val = ((i - x_min) / (x_max - x_min) * (sim_max - sim_min)) + sim_min
        new_x.append(new_x_val)

        new_y_val = ((j - y_min) / (y_max - y_min) * (sim_max - sim_min)) + sim_min
        new_y.append(new_y_val)

    # Return the List of coordinates
    returnList = []
    for i in range(0, len(x_axis)):
        coord = (new_x[i], new_y[i])
        returnList.append(coord)
    return returnList