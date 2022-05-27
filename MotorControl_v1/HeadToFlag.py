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

RobotMotion = RobotMotion()

#Make green limits a global variable 
green_lower = np.array([50, 112, 28], np.uint8)
green_upper = np.array([100,230,64], np.uint8)
kernal = np.ones((5, 5), "uint8")

#Function to see if position is within a tolerance wrt to goal 
def WithinTolerance(tol, posX, posY,  goalX, goalY):
    if(posX > goalX - tol and posX < goalX + tol and posY > goalY - tol and posY < goalY + tol):
            return True
    else:   return False

##To read only an image for cv2 
def captureImage(camera):
    _, img = camera.read()
    return img

def getFlagCenter(imageFrame):
    global green_lower
    global green_upper
    global kernal

    #initialize the center to null 
    cx = None
    cy = None
    avgX = None
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
        
        #Find the center of the box
        #TODO: YOU CAN REMOVE MOMENTS IF AVG IF EXT TOP AND BOT is GOOD
        M = cv2.moments(c)
        extTop = tuple(c[c[:, :, 1].argmin()][0])        
        extBot = tuple(c[c[:, :, 1].argmax()][0])        

        avgX = (extTop[0]+extBot[0])/2
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
    
    return cx,cy,avgX

withinTol = 0
def centerWithFlag(Fx):
    global withinTol
    kP = .525 #0.625
    screen_center_x = 320 #center pixel of screen
    if (Fx != None): #if flag detected
        if (WithinTolerance(30,Fx,Fx,320,320)): #if flag centered
            RobotMotion.stopRobot()
            withinTol = withinTol + 1
            print("WitinTol:",withinTol)
            if(withinTol == 6):
                return True
        else: #else flag not centered
            #implement a P loop
    
            # Normalize error 
            err = float(abs(Fx-320)) #/320
            print("err: ",err)
            if(err < 100):
                kP = 0.8
            if(err < 60):
                kP = 1
            if(err < 40):
                kP = 2
            print("kP: ", kP)
            Rspeed = kP*err
            if Rspeed > 100:
                Rspeed = 100
            print("Speed: ", Rspeed)
            #TODO: sleeping for 0.625 second rotates around 25 degrees this needs to be tuned
            if(Fx > screen_center_x):
                #crab right or turn CW
                RobotMotion.CW(Rspeed)
            
            elif(Fx < screen_center_x):
                #crab left or turn CCW
                RobotMotion.CCW(Rspeed)

            #adjust robot and return that you are not centered, and check if 
            # you are centered in the next loop
            return False

    else:
        #if no flag found, then rotate 20 degrees and repeat main 
        RobotMotion.CW()
        return False

def main():
    centered = False
    while (not centered):
        #capture image 
        imageFrame = captureImage(camera)
        Fx,Fy, newX = getFlagCenter(imageFrame)
        #print(Fx)
        #cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        #cv2.imshow("Mask", green_mask)
        #keep going in a loop until you are centered with flag
        centered = centerWithFlag(newX)
        
    
if __name__ == "__main__":
    
    #Create a camera object and initialize it
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    #TODO: you can remove camera flip, only there for debugging
    time.sleep(2)
    main()