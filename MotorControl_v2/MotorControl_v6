#! /usr/bin/python
import sys
from open_motor_serial import open_motor
import serial
import KeyPressModule as kp
import pygame
import time
import math
from MarvelMind import MarvelmindHedge
import cv2
import numpy as np
from threading import Thread, Event
from RobotMotion import RobotMotion


#ARDUINO SERIAL INIT.
#######################################################################
ser = serial.Serial('/dev/ttyACM0_Arduino', 57600, timeout=1)
ser.reset_input_buffer()
#######################################################################

#import keyboard
sys.path.append("../src/open_motor/")

#Initialize Robot Motion class which controls robots
RobotMotion = RobotMotion()

#Initialize Robot Mode
mode = "RC"

#Robot States
######################
Avoiding = True #We start in avoiding mode, and turn off after avoiding obstacles
FindingFlag = False
GettingFlag = False
blocked = False
######################

#Sub States
############################################
hallwayCenterGoing = False
flagZone = False
centered_w_Flag = False
captured = False
hallwayCenterComin = False
Delivered = False
############################################

#Initialize Camera Variables
withinTol = 0
#Make green limits a global variable 
green_lower = np.array([50, 112, 28], np.uint8)
green_upper = np.array([100,230,64], np.uint8)
kernal = np.ones((5, 5), "uint8")

#initialize number of flags delivered
FlagsDelivered = 0


#Initialize pygame keyboard
kp.init()

# Function thread to get Ultrasonic data from Arduino 
def get_USDATA():
    global blocked
    #TODO: Would there be a race condition between the .wait(), the Set(), and clear()?
    #  clear the event at beggining of thread 
    while True:
        #If there are incoming bits 
        if  ser.inWaiting()>0: 
            #read the data which is a string in the form: US1,US2,US3,US4
            dataRaw = ser.readline().decode('utf-8').rstrip()
            #Separate into a list with "," as separator
            dataList = dataRaw.split(",")
            US1 = float(dataList[0])
            US2 = float(dataList[1])
            US3 = float(dataList[2])
            US4 = float(dataList[3])   
            #print(US1, US2, US3, US4)

            if(US1 < 55 or US2 < 100 or US3 < 100 or US4 < 55 ):
                blocked = True
                #Turn towards left or right based on which reads a further distance
                if(US5 > US6):
                    RobotMotion.right()
                else:
                    RobotMotion.left()

            elif(US1 > 55 and US2 > 100 and US3 > 100 and US4 > 55 ):
                blocked = False
            
            return US2, US3 # if there is a reading return front two
        
        return None # if there is not reading return none
    
#Function to check the mode 
def modeSwitch():
    global mode
    for event in pygame.event.get():
        #if key is pressed move motor based on key
        if event.type == pygame.KEYDOWN:
            
            if kp.getKey('1'):
                mode = "RC"
                
            if kp.getKey('2'):
                mode = "Confirmation"
            
            elif kp.getKey('3'):
                mode = "Scout"
                
            else:
                mode = mode

#Function to find heading for the first time 
def FindHeading(goalX, goalY, timeSleepForward=2):
    #Get Original Position
    x1 = hedge.position()[1]
    y1 = hedge.position()[2]
    hedge.dataEvent.wait(1)
    hedge.dataEvent.clear()
    
    #Make it run for a little bit
    RobotMotion.forward()
    time.sleep(timeSleepForward)
    RobotMotion.stop()
    #print("OG POS: ", x1, y1)

    #Get New position
    time.sleep(5)
    x2 = hedge.position()[1]
    y2 = hedge.position()[2]
    hedge.dataEvent.wait(1)
    hedge.dataEvent.clear()
    #print("New POS: ", x2, y2)

    #define vector between old pos and goal
    A = np.array([x2-x1, y2-y1])
    #print(A)
    
    #Define vector between new pos and goal
    B = np.array([goalX-x2, goalY-y2])
    #print(B)
    
    #find angle between the two
    angTurnRad = np.arccos(np.dot(A,B)/(np.linalg.norm(A)*np.linalg.norm(B)))
    #print(angTurnRad)
     
    #Get it in degrees
    angTurn = np.degrees(angTurnRad)
    #print(angTurn)
    #print("I need to turn: ", angTurn, " degrees to get to target")
    #print("I need to turn clockwise for: ", turningTime, "s to align myself with goal")

    #TODO: You dont need any of this. Its here only for debugging and testing
    FullRotationTime = 9
    turningTime = angTurn * FullRotationTime/360
    PPR = angTurn * 1425.1/360 # this is the PPR value that we need to add/sub to each motor

    direc = np.cross(A,B)
    #print(direc)
    
    #sleep buffer
    #TODO: See if you can reduce the time.sleep time. Maybe 0.025 or something like that is enough
    time.sleep(1)

    #Rotate as needed 
    RobotMotion.rotateDegrees(angTurn, direc)

    return x2, y2

#Function to find heading if you already have your position 1 and position 2 coordinates
def UpdateHeading(x1, y1, x2, y2, goalX, goalY):

    #define vector between old pos and goal
    A = np.array([x2-x1, y2-y1])
    #print(A)
    
    #Define vector between new pos and goal
    B = np.array([goalX-x2, goalY-y2])
    #print(B)
    
    #find angle between the two
    angTurnRad = np.arccos(np.dot(A,B)/(np.linalg.norm(A)*np.linalg.norm(B)))
    #print(angTurnRad)
     
    #Get it in degrees
    angTurn = np.degrees(angTurnRad)
    print(angTurn)
    
    FullRotationTime = 9
    turningTime = angTurn * FullRotationTime/360
    PPR = angTurn * 1425.1/360 # this is the PPR value that we need to add/sub to each motor
    
    #print("I need to turn: ", angTurn, " degrees to get to target")
    #print("I need to turn clockwise for: ", turningTime, "s to align myself with goal")

    direc = np.cross(A,B)
    print(direc)
    
    #sleep buffer
    time.sleep(1)
    
    if(direc >= 0):
     #Turn clockwise
       # print("turning CW")
        RobotMotion.CW()
        
    else:
        #turn CCW
        #print("CCW")
        RobotMotion.CCW()

    time.sleep(turningTime)
    RobotMotion.stop()
    #print("Stopped")

#Function to see if position is within a tolerance wrt to goal 
def WithinTolerance(tol, posX, posY,  goalX, goalY):
    if(posX > goalX - tol and posX < goalX + tol and posY > goalY - tol and posY < goalY + tol):
            return True
    else:   return False

#Has tolerance for both x and y 
def WithinTolerance(xtol, ytol, posX, posY, goalX, goalY):
    if(posX > goalX - xtol and posX < goalX + xtol and posY > goalY - ytol and posY < goalY + ytol):
            return True
    else:   return False

def get_Position():
    x1 = hedge.position()[1]
    y1 = hedge.position()[2]
    hedge.dataEvent.wait(1)
    hedge.dataEvent.clear()
    return x1, y1

#Function to see if position is within a tolerance wrt to goal 
def WithinTolerance(tol, posX, posY,  goalX, goalY):
    if(posX > goalX - tol and posX < goalX + tol and posY > goalY - tol and posY < goalY + tol):
            return True
    else:   return False

def closeGrabbingClaw():
    ser.write(b"Close Servo_grab\n")

def openGrabbingClaw():
    ser.write(b"Open Servo_grab\n")

def liftClaw():
    ser.write(b"Open Servo_lift\n")

def lowerClaw():
     ser.write(b"Close Servo_lift\n")

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

def centerWithFlag(Fx):
    global withinTol
    kP = .525 #0.625
    screen_center_x = 320 #center pixel of screen

    if (Fx != None): #if flag detected
        if (WithinTolerance(30,Fx,Fx,320,320)): #if flag centered
            RobotMotion.stop()
            withinTol = withinTol + 1
            print("WitinTol:",withinTol)
            if(withinTol == 6):
                withinTol = 0 #reset withinTol
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
        RobotMotion.CW(150)
        return False

def resetSubStates():
    global hallwayCenterGoing
    global flagZone
    global centered_w_Flag
    global captured
    global hallwayCenterComing
    global Delivered

    hallwayCenterGoing = False
    flagZone = False
    centered_w_Flag = False
    captured = False
    hallwayCenterComing = False
    Delivered = False

# RC mode funciton
def RCMODE(): 
    for event in pygame.event.get():
        #if key is pressed move motor based on key
        if event.type == pygame.KEYDOWN:
            
            if kp.getKey('UP'):
                RobotMotion.forward()

                #Backward
            elif kp.getKey('DOWN'):
                RobotMotion.backward()

                #Right
            elif kp.getKey('RIGHT'):
                RobotMotion.right()

                #Left
            elif kp.getKey('LEFT'):
                RobotMotion.left()

            #topright
            elif kp.getKey('e'): #SEE IF YOU CAN DO 'forward' AND 'RIGHT'
                RobotMotion.topright()

            #topleft
            elif kp.getKey('q'): #SEE IF YOU CAN DO 'LEFT' AND 'RIGHT'
                RobotMotion.topleft()

            #bottomright
            elif kp.getKey('c'): #SEE IF YOU CAN DO 'forward' AND 'RIGHT'
                RobotMotion.bottomright()

            #bottomleft
            elif kp.getKey('z'): #SEE IF YOU CAN DO 'LEFT' AND 'RIGHT'
                RobotMotion.bottomleft()
                
            #CCW rotate
            elif kp.getKey('d'):
                RobotMotion.CCW()

            #CW rotate
            elif kp.getKey('a'):
                RobotMotion.CW()

            #Stop
            elif kp.getKey('s'):
                RobotMotion.stop()
            
            ##########################TTTTTTTTESSSSSSSSTINGGGG FUNCITON##########3
            #elif kp.getKey('t'):

                
    #Servo Motor Strings

            elif kp.getKey('o'):
                ser.write(b"Open Servo_grab\n")
                
            elif kp.getKey('p'):
                ser.write(b"Close Servo_grab\n")
            
            #lifts up
            elif kp.getKey('k'):
                ser.write(b"Open Servo_lift\n")
            
            #Lift down
            elif kp.getKey('l'):
                ser.write(b"Close Servo_lift\n")

    # Mode Switch Check
            elif kp.getKey('2'):
                mode = "Confirmation"
                
            elif kp.getKey('3'):
                mode = "Scout"
                
            #If key is lifted stop motors
        elif event.type == pygame.KEYUP:
            RobotMotion.stop()
    
def main():
    global mode

             #LEVEL 1
####################################################################
    if mode == "RC":
        ser.write(b"RC Mode\n")
        print("I am in RC Mode :)")
        #Mode is switched inside RCMODE function
        #The while loop is used so that we dont loop over main and 
        # keep writing that we are in RC Mode and sending to serial    
        while mode == "RC":
            RCMODE()    
                    
                    
    #LEVEL 2
####################################################################
    elif mode == "Confirmation":
        #MAIN ROBOT STATES
        global Avoiding
        global FindingFlag
        global GettingFlag
        global blocked

        #SUB STATES
        global hallwayCenterGoing
        global flagZone
        global centered_w_Flag
        global captured
        global hallwayCenterComing
        global Delivered

        #Number of Flags delivered
        global FlagsDelivered

        ser.write(b"Confirmation Mode\n")
        time.sleep(0.25)
        openGrabbingClaw()
        time.sleep(1)
        liftClaw()
        print("I am in Confirmation Mode :)")

        while(mode == "Confirmation"):
            
            if(Avoiding and not blocked and mode == "Confirmation"):
                RobotMotion.forward(200)
                x1, y1 = get_Position()
                if(x1 > 14.5):
                    Avoiding = False
                    FindingFlag = True

            if(FindingFlag and mode == "Confirmation"):
                
                #Go to center of hallway
                while(not hallwayCenterGoing and mode == "Confirmation"):
                    x1, y1 = get_Position()
                    if(x1 > 14.53 and y1 < 0.936 and mode == "Confirmation"):
                        hallwayCenterGoing = True
                    elif(mode == "Confirmation"):
                        FindHeading(15.65, 0.82)
                        RobotMotion.forward(200)
                        time.sleep(1.5) #TODO: Tune this number
                        RobotMotion.stop()
                        time.sleep(5) #TODO Tune htis number
                
                #go to the Flag zone
                while(not flagZone and mode == "Confirmation"):
                    x1, y1 = get_Position()
                    if(x1 > 16.6 and mode == "Confirmation"):
                        flagZone = True
                    elif(mode == "Confirmation"):
                        FindHeading(16.6,0.936)
                        RobotMotion.forward(200)
                        time.sleep(1.5) #TODO: Tune this number
                        RobotMotion.stop()
                        time.sleep(5) #TODO Tune htis number
                    
                #when in goal zone find the flag:
                while (not centered_w_Flag and mode == "Confirmation"):
                    #capture image 
                    imageFrame = captureImage(camera)
                    Fx,Fy, newX = getFlagCenter(imageFrame)
                    #print(Fx)
                    #cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
                    #cv2.imshow("Mask", green_mask)
                    #keep going in a loop until you are centered with flag
                    centered_w_Flag = centerWithFlag(newX)
                
                #Capture the flag once identified
                while (not captured and mode == "Confirmation"):
                    RobotMotion.forward(100)
                    US2, US3 = get_USDATA()
                    if((US2 <= 7 or US3 <= 7) and mode == "Confirmation"): #TODO Tune this number
                        RobotMotion.stop()
                        captured = True
                        lowerClaw()
                        time.sleep(1)
                        closeGrabbingClaw()
                        time.sleep(1)
                        liftClaw()
                        FindingFlag = False
                        GettingFlag = True
            
            if(GettingFlag and mode == "Confirmation"):

                while(not hallwayCenterComing and mode == "Confirmation"):
                    x1, y1 = get_Position()
                    if(x1 < 16.2 and mode == "Confirmation"):
                        hallwayCenterComing = True
                    elif(mode == "Confirmation"):
                        FindHeading(15.65,0.82)
                        RobotMotion.forward(200)
                        time.sleep(1.5) #TODO: Tune this number
                        RobotMotion.stop()
                        time.sleep(5) #TODO Tune htis number

                #To to goal Zone:
                while(not Delivered and mode == "Confirmation"):
                    x1, y1 = get_Position()
                    if(WithinTolerance(0.5,x1,y1,15.43,6.8) and mode == "Confirmation"):
                        RobotMotion.stop()
                        lowerClaw()
                        time.sleep(1)
                        openGrabbingClaw()
                        Delivered = True
                        FlagsDelivered += 1
                        if(FlagsDelivered == 2 and mode == "Confirmation"):
                            mode = "RC"
                        elif(mode == "Confirmation"):
                            RobotMotion.backward()
                            time.sleep(1)
                            RobotMotion.stop()
                            #If you havent found both flags, return to finding flag state
                            resetSubStates()
                            GettingFlag = False
                            FindingFlag = True
                    elif(mode == "Confirmation"):
                        FindHeading(15.43,6.83)
                        RobotMotion.forward(200)
                        time.sleep(2)
                        RobotMotion.stop()
                        time.sleep(5)
            
            if(blocked and mode == "Confirmation"):
                while(blocked and mode == "Confirmation"): pass
                    
            #LEVEL 3
####################################################################
    elif mode == "Scout":
        ser.write(b"Scout Mode\n")
        print("I am in Scout Mode :)")
        while(mode == "Scout"):
            
            #Check if user switched mode        
            modeSwitch()
            
        
if __name__ == "__main__":

    #Create a camera object and initialize it
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1) # make buffer size = 1 

    #Make sure marvelmind is connected after connecting the motor controller
    #since we use ACM1 and not ACM0
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1_MarvelMind", adr=None, debug=False) # create MarvelmindHedge thread

    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread
    
    #Create US thread with the get_USDATA function
    US_thread = Thread(target=get_USDATA, args=(), daemon=True)
    #Start US thread
    US_thread.start()

    #Create thread to change the mode of the robot
    mode_thread = Thread(target=modeSwitch, args=(), daemon=True)
    #Start Mode thread
    mode_thread.start()

    while True:
        main()
        