import pygame, sys
from pygame.locals import *
import math
from Kalman_sim_math import generate_curved_path, determinePath, discoveryDroneWayPoint

from Kalman_sim_prediction import linearPrediction, linearPredictionVector

from Kalman_class import kalman_function1

BLUE = ( 0, 0, 255)
RED = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
FPS = 10
fpsClock = pygame.time.Clock()


class Drone:
    def __init__(self, name, xPos, yPos, display):
        self.name = name
        self.xPos = xPos
        self.yPos = yPos
        self.display = pygame.image.load(display)
        self.coords = (xPos, yPos)

    def update(self, DISPLAYSURF):
        DISPLAYSURF.blit(self.display, self.coords)
        #print(self.coords)



def makeText(text, color, bgcolor, top, left):
    # create the Surface and Rect objects for some text.
    BASICFONT = pygame.font.Font('freesansbold.ttf', 12)

    textSurf = BASICFONT.render(text, True, color, bgcolor)
    textRect = textSurf.get_rect()
    textRect.topleft = (top, left)
    return (textSurf, textRect)



def initilizeSimulation(DISPLAYSURF):

    pygame.init()

    
    pygame.display.set_caption('Kalman Filter Simulation')


    ################ Init Drone Class ############################
    rougeDroneXPos = 100
    rougeDroneYPos = 700
    rougeDrone = Drone("RougeDrone", 100, 700, 'rouge_drone1.png')
    discoveryDrone = Drone("DiscoveryDrone", 650, 700, 'discovery_drone1.png')
    predictedRougeDrone = Drone("PredictedRougeDrone", 400, 400, 'predicted_drone2.png')
    predictedRougeDroneFinal = Drone("PredictedRougeDroneFinal", 400, 400, 'predicted_drone2.png')
    radar = Drone("Radar", 725, 650, 'radar.png')

    # Import png files
    rougeDroneImg = pygame.image.load('rouge_drone1.png')
    discoveryDroneImg = pygame.image.load('discovery_drone1.png')
    radarImg = pygame.image.load('radar.png')
    
    predictedDroneImg = pygame.image.load('predicted_drone2.png')

    discoveryDronePos = (650, 700)
    radarPos = (725, 650)
    rougeDronePos = (rougeDroneXPos, rougeDroneYPos)
    original_signalXPos = 675
    original_signalYPos = 625


    DISPLAYSURF.blit(rougeDrone.display, rougeDrone.coords)
    DISPLAYSURF.blit(discoveryDrone.display, discoveryDrone.coords)
    DISPLAYSURF.blit(radar.display, radar.coords)

    objectList = [rougeDrone, predictedRougeDrone, discoveryDrone, predictedRougeDroneFinal, radar]

    return objectList


    ##################### End of Init ###################################
def draw_line_with_slope(surface, point, slope, length):
    x0, y0 = point
    # Calculate the end points of the line
    x1 = x0 - length # Go left
    #x1 = x0 + length # Go Right
    y1 = y0 + slope * length

    # Draw the line
    pygame.draw.line(surface, BLUE, (x0, y0), (x1, y1), 3)

def draw_arrow(surface, color, start, end, width=3, arrow_size=10):
    # Draw the line
    pygame.draw.line(surface, color, start, end, width)

    # Calculate the angle of the line
    angle = math.atan2(end[1] - start[1], end[0] - start[0])

    # Calculate the points of the arrowhead
    point1 = (end[0] - arrow_size * math.cos(angle - math.pi / 6),
              end[1] - arrow_size * math.sin(angle - math.pi / 6))
    point2 = (end[0] - arrow_size * math.cos(angle + math.pi / 6),
              end[1] - arrow_size * math.sin(angle + math.pi / 6))

    # Draw the arrowhead
    pygame.draw.polygon(surface, color, [end, point1, point2])

def point_to_line_distance(px, py, x1, y1, x2, y2):
    # Line equation: Ax + By + C = 0
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2
    # Distance formula
    return abs(A * px + B * py + C) / math.sqrt(A**2 + B**2)

def draw_decay_line(surface, start_point, line_start, line_end, decay_factor):
    x0, y0 = start_point
    x1, y1 = line_start
    x2, y2 = line_end

    # Calculate the distance to the line
    distance = point_to_line_distance(x0, y0, x1, y1, x2, y2)

    # Exponential decay calculation
    decay_length = distance * decay_factor
    if decay_length > 0:
        # Find the unit vector towards the line
        A = y2 - y1
        B = x1 - x2
        line_length = math.sqrt(A**2 + B**2)
        unit_vector = (A / line_length, B / line_length)

        # New end point of the decaying line
        decay_x = x0 + decay_length * unit_vector[0]
        decay_y = y0 + decay_length * unit_vector[1]

        # Draw the decay line
        pygame.draw.line(surface, RED, start_point, (decay_x, decay_y), 3)


def update_graphics(DISPLAYSURF, objectList):

    DISPLAYSURF.fill((0, 0, 0))  # Fill with black

    rougeDronePosText = 'Rouge Drone Position: ' + str(objectList[0].coords)
    RougeDronePos_SURF, RougeDronePos_RECT = makeText(rougeDronePosText, WHITE, BLACK, 500, 100)
    DISPLAYSURF.blit(RougeDronePos_SURF, RougeDronePos_RECT)

    discoveryDronePredictionText = 'Predicted Rouge Drone to be at: ' + str(objectList[1].coords)
    RESET_SURF2, RESET_RECT2 = makeText(discoveryDronePredictionText, WHITE, BLACK, 500, 150)
    DISPLAYSURF.blit(RESET_SURF2, RESET_RECT2)

    for drone in objectList:
        drone.update(DISPLAYSURF)


    # Draw vector of rouge drone predicted Path
    line_width = 3
    arrow_baseX = objectList[1].coords[0] + 20
    arrow_baseY = objectList[1].coords[1] + 20
    arrow_tipX = objectList[3].coords[0] + 20
    arrow_tipY = objectList[3].coords[1] + 40
    draw_arrow(DISPLAYSURF, WHITE, (arrow_baseX, arrow_baseY), (arrow_tipX, arrow_tipY), line_width)
    #pygame.draw.line(DISPLAYSURF, WHITE, objectList[1].coords, objectList[3].coords, line_width)



def run_graphics():
    DISPLAYSURF = pygame.display.set_mode((800, 800))
    objectList = initilizeSimulation(DISPLAYSURF)

    ############################# SET DESIRED PATH ######################################
    ## Straight Path: "S"
    ## Curved Path: "C"
    ## CSV Path: "V"
    desiredPath = "S"
    filename = "radarDataOct04_2.csv"

    rougeDronePathNew = determinePath(desiredPath, filename, "R")

    # Update using the Kalman filter
    rougeDronePathNew = kalman_function1(rougeDronePathNew, 1)
    
    ################## Temporary Discover Drone Path ###############################
    tempPath = determinePath(desiredPath, filename, "D")


    ################################### SET VIDEO OR STEP #####################################
    ## Continuous: False
    ## Step: True
    pressed_on = False


    pressed = False

    pressed_video = False

    pathIndex = 0


    signalXPos = (objectList[3].xPos - 50)
    signalYPos = (objectList[3].yPos - 25)

    while True: # main game loop

        # Predicted Drone
        previousMeasurement = rougeDronePathNew[pathIndex - 1]
        currentMeasurement = rougeDronePathNew[pathIndex]
        predictedDronePos = linearPrediction(previousMeasurement, currentMeasurement)
        predicedRDronePath = linearPredictionVector(previousMeasurement, currentMeasurement)
        predictedDronePosFinal = linearPredictionVector(previousMeasurement, currentMeasurement)[1]
    

        if pressed == True and pathIndex < len(rougeDronePathNew) - 1 and pressed_on == True:

            pathIndex = pathIndex + 1

            update_graphics(DISPLAYSURF, objectList)
        
            # Move signal to rouge drone and back
            signalDroneYDiff = signalYPos - rougeDronePathNew[pathIndex][1]
            signalDroneXDiff = signalXPos - rougeDronePathNew[pathIndex][0]
            signalImg = pygame.image.load('signal.png')
            DISPLAYSURF.blit(signalImg, (signalXPos, signalYPos))
            while (signalDroneXDiff >= 100):
                signalDroneYDiff = signalYPos - rougeDronePathNew[pathIndex][1]
                signalDroneXDiff = signalXPos - rougeDronePathNew[pathIndex][0]
                signalXPos = signalXPos - 25
                signalYPos = signalYPos - (signalDroneYDiff / (signalDroneXDiff / 25))
                

                objectList[0].coords = rougeDronePathNew[pathIndex]
                objectList[1].coords = predictedDronePos
                update_graphics(DISPLAYSURF, objectList)

                DISPLAYSURF.blit(signalImg, (signalXPos, signalYPos))
                pygame.display.update()
                fpsClock.tick(FPS * 4)

            # Return Signal back
            o_signalDroneXDiff = (objectList[3].xPos - 50) - signalXPos
            o_signalDroneYDiff = (objectList[3].yPos - 25) - signalYPos
            while (o_signalDroneXDiff >= 50):
                o_signalDroneXDiff = (objectList[3].xPos - 50) - signalXPos
                o_signalDroneYDiff = (objectList[3].yPos - 25) - signalYPos
                signalXPos = signalXPos + 25
                signalYPos = signalYPos + (o_signalDroneYDiff / (o_signalDroneXDiff / 25))
                

                update_graphics(DISPLAYSURF, objectList)

                if (o_signalDroneXDiff >= 50):
                    DISPLAYSURF.blit(signalImg, (signalXPos, signalYPos))
                pygame.display.update()
                fpsClock.tick(FPS * 4)

            
            pressed = False


        elif pressed_on == False and pressed_video == True:
            pathIndex = pathIndex + 1


            # Get new path for the discovery drone
            #objectList[2].coords = tempPath[pathIndex]
            objectList[2].coords = discoveryDroneWayPoint(objectList[2].coords, currentMeasurement, predictedDronePos)

            objectList[0].coords = rougeDronePathNew[pathIndex]
            objectList[1].coords = predictedDronePos

            objectList[3].coords = predictedDronePosFinal

            update_graphics(DISPLAYSURF, objectList)

            
            fpsClock.tick(FPS * 2)



        for event in pygame.event.get():

            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if (event.type == pygame.MOUSEBUTTONDOWN):
                pressed = True
                pressed_video = True
        pygame.display.update()
        fpsClock.tick(FPS)
