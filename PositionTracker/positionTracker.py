# records the motion of the robot
# imports
import cv2 as cv # OpenCV library
import numpy as np # Numpy library for scientific computing
import matplotlib.pyplot as plt # Matplotlib library for plotting
import time
from PIL import ImageGrab

# grab initial screen to draw on
mapScreenOG =  ImageGrab.grab()
mapScreenOG =   np.array(mapScreenOG.getdata(),dtype='uint8').reshape((mapScreenOG.size[1],mapScreenOG.size[0],3)) 
mapScreenOG = mapScreenOG[100:950,400:1550]
#cv.imshow('robot tracking',mapScreen)

# webots map is within:
# top left      = (400,100)
# top right     = (1750, 100)
# bottom left   = (400, 950)
# bottom right  = (1750,950)

count = 0
while (count < 200000):
    # find the robot using HSV thresholding
    mapScreen = ImageGrab.grab()
    mapScreen = np.array(mapScreen.getdata(),dtype='uint8').reshape((mapScreen.size[1],mapScreen.size[0],3)) 
    mapScreen = mapScreen[100:950,400:1550]

    hsv_min = np.array([26,3,87])
    hsv_max = np.array([109,108,171])
    mapHSV = cv.cvtColor(mapScreen, cv.COLOR_RGB2HSV)
    mapHSV = cv.inRange(mapHSV, hsv_min, hsv_max)

    # draw dot at centre of robot
    robot_isolated = cv.HoughCircles(mapHSV, cv.HOUGH_GRADIENT, 1, minDist=300, param1=15, param2=7, minRadius=10, maxRadius=50)
    robot_centre = None
    if robot_isolated is not None:
        robot_isolated = np.round(robot_isolated[0, :]).astype("int")
        for (x,y,r) in robot_isolated:
            #print("adding new centroid")
            robot_centre = (x-12,y+11)
            cv.circle(mapScreenOG, robot_centre, 3, (255,0,0), 5)

    # display updated image
    cv.imshow('robot tracking',mapScreenOG)
    if cv.waitKey(25) & 0xFF == ord('q'):
        cv.destroyAllWindows()
        break
    #time.sleep(0.01)

    count += 1

print("program finished")