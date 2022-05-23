import sys
from RobotMotion import RobotMotion
import serial
import KeyPressModule as kp
import pygame
import time
import math
from MarvelMind import MarvelmindHedge
from picamera import PiCamera
import numpy as np
from queue import Queue
from threading import Thread, Event
import numpy as np
import cv2

baudRate = 115200

#import keyboard
sys.path.append("../src/open_motor/")

# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

#comms = open_motor()
#comms.init_serial_port(port,baudRate,0.5)

#Initialize Robot Mode
mode = "RC"

#Initialize pygame keyboard
kp.init()

RobotMotion = RobotMotion()

#Make green limits a global variable 
green_lower = np.array([45, 55, 0], np.uint8)
green_upper = np.array([120, 150, 70], np.uint8)
kernal = np.ones((5, 5), "uint8")


#Motor Control function
def MotorRun(Mot0, Mot1, Mot2, Mot3):
    topright = Mot1*speed
    topleft = Mot2*speed
    bottomright = Mot0*speed
    bottomleft = Mot3*speed

    #Adjusting motor direction
    bottomright = -bottomright
    topleft = -topleft
    bottomleft = -bottomleft
    
    #Send the signals to 
    comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)

#Rotate in place by indicating degrees and direction.
#  direction >= 0 ------> CW
#  direction < 0  ------> CCW
def rotateDegrees(degrees, direction=0):
    FullRotationTime = 9
    turningTime = degrees * FullRotationTime/360
   
    if(direction >= 0):
     #Turn clockwise
        RobotMotion.CW()
        
    else:
        #turn CCW
        RobotMotion.CCW()

    time.sleep(turningTime)
    RobotMotion.stopRobot()

#Function to see if position is within a tolerance wrt to goal 
def WithinTolerance(tol, posX, posY,  goalX, goalY):
    if(posX > goalX - tol and posX < goalX + tol and posY > goalY - tol and posY < goalY + tol):
            return True
    else:   return False

##To read only an image for cv2 
def captureImage():
    img = cv2.imread('/home/pi/Pictures/PresImage.jpg')
    cv2.imshow("image", img)
    cv2.waitKey(0)
    return img

def getFlagCenter():
    global green_lower
    global green_upper
    global kernal

    #initialize the center to null 
    cx = None
    cy = None

    #Blue the camera to reduce the noise
    for i in range(7):
        imageFrame = cv2.GaussianBlur(imageFrame,(5,5),0)

    # Convert the imageFrame in 
    # BGR(RGB color space) to 
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for green color and 
    # define mask
    #green_lower and upper are global variables that remain constant
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
      
    # Apply a dilate filter to consolidate green pixels together
    green_mask - cv2.erode(green_mask, kernal, iterations =2)
    green_mask = cv2.dilate(green_mask, kernal, iterations = 4)
  
    #Perform filter operation by anding frame using green_mask
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask = green_mask)
     
  
    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    
    
    #Only Draw the Largest Countour
    if len(contours) != 0:
        # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)  
        imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                   (x + w, y + h),
                                   (0, 255, 0), 2)
        
        #Find the center of the box 
        M = cv2.moments(c)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        
        cv2.circle(imageFrame,(cx,cy),7,(255,255,255),-1)           
        cv2.putText(imageFrame, "Green Colour", (cx-20,cy-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    1.0, (255,255,255))
               
    # Program Termination
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    cv2.imshow("Mask", green_mask)

    return cx,cy

def centerWithFlag(Fx):
    if (Fx != None):
        if (WithinTolerance(30,Fx,Fx,320,320)):
            RobotMotion.stopRobot()
            return True
        else:
            #implement a P loop
            
            # Normalize error 
            err = abs(Fx-320)/320
            #pass error to the time sleep function to sleep more or less 

            #TODO: sleeping for 1 second rotates around 40 degrees this needs to be tuned
            if(Fx > 320):
                #crab right or turn CW
                RobotMotion.CW()
                time.sleep(1*err)
                RobotMotion.stopRobot()
            
            elif(Fx < 320):
                #crab left or turn CCW
                RobotMotion.CCW()
                time.sleep(1*err)
                RobotMotion.stopRobot()

            #adjust robot and return that you are not centered, and check if 
            # you are centered in the next loop
            return False

    else:
        #if no flag found, then rotate 40 degrees and repeat main 
        rotateDegrees()
        return False


def main():
    #capture image 
    imageFrame = captureImage()
    Fx,Fy = getFlagCenter()
    centered = False
    #keep going in a loop until you are centered with flag
    while (not centered):
        centered = centerWithFlag(Fx)
    
if __name__ == "__main__":
    #Create a camera object and initialize it
    camera = PiCamera()
    #TODO: you can remove camera flip, only there for debugging
    camera.vflip = True
    time.sleep(2)
    main()