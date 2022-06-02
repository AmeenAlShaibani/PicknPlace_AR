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
from RobotMotion_VELOCITY import RobotMotion

#Issues seen and possible fixes
##################################################################################
#There is a possibility that when battery is low, communication becomes slower
# , so you cant execute multiple movement commands one after the other

#if you are sending the same command but not the same thing happening, buffer for a long time
#probabbly a threading issue

#When sensor is not reading, disconnext sensor, reconnect, disconnect arduino and reconnect

#ARDUINO SERIAL INIT.
#######################################################################
#Try changing to 115200
ser = serial.Serial('/dev/ttyACM0_Arduino', 115200, timeout=1)
time.sleep(0.1)
if(ser.isOpen()):
    print("Arduino Serial Port Opened")
ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
#######################################################################

#import keyboard
sys.path.append("../src/open_motor/")

#Initialize Robot Motion class which controls robots
RobotMotion = RobotMotion()

#Initialize Robot Mode
mode = "RC"

#Robot States
######################
Avoiding = False #We start in avoiding mode, and turn off after avoiding obstacles
FindingFlag = True
GettingFlag = False
blocked = False
######################

#Sub States
############################################
hallwayCenterGoing = True
flagZone = True
centered_w_Flag = False
captured = False
hallwayCenterComing = False
Delivered = False

flagZone_interrupt = False # interrupt used to stop threads when needed
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

global FL
global FR

#global time variables
global NowTime

#global current Position variables
global x1, y1

#Starts the Ultrasonic and ModeSwitch threads:
def startThreads():
    #Create US thread with the get_USDATA function
    US_thread = Thread(target=get_USDATA, args=(), daemon=True)
    #Start US thread
    US_thread.start()

    #Create thread to change the mode of the robot
    mode_thread = Thread(target=modeSwitch, args=(), daemon=True)
    #Start Mode thread
    mode_thread.start()

# Function thread to get Ultrasonic data from Arduino 
def get_USDATA():
    global blocked
    global FL
    global FR
    MovingRight = False
    MovingLeft = False
    #TODO: Would there be a race condition between the .wait(), the Set(), and clear()?
    while (not flagZone_interrupt):#  clear the event at beggining of thread 
        while ser.inWaiting()==0: pass
            #If there are incoming bits 
        if  ser.inWaiting() > 0:
            #read the data which is a string in the form: US1,US2,US3,US4
            dataRaw = ser.readline().decode('utf-8').rstrip()
            #Separate into a list with "," as separator
            dataList = dataRaw.split(",")
            #These are flipped in the arduino code
            SL = float(dataList[0])
            EL = float(dataList[1])
            FL = float(dataList[2])
            FR = float(dataList[3])
            ER = float(dataList[4])   
            SR = float(dataList[5])   

            ser.flushInput()
            #print(SR, ER, FR, FL, EL, SL)
            #70 130 130 70
            #55 90 90 55
            if((ER < 100 or FR < 100 or FL < 100 or EL < 100) and mode == "Confirmation" and Avoiding):
                blocked = True
                #Turn towards left or right based on which reads a further distance
                if((SR > SL) and not MovingLeft):
                    MovingRight = True
                    RobotMotion.right(200) # was 150
                elif(SR < SL and not MovingRight):
                    MovingLeft = True
                    RobotMotion.left(200)

            elif((ER > 100 and FR > 100 and FL > 100 and EL > 100) and mode == "Confirmation" and Avoiding):
                MovingRight = False
                MovingLeft = False
                blocked = False
            
            #return FR, FL # if there is a reading return front two
    
    #return None # if there is not reading return none
    
#Function to check the mode 
def modeSwitch():
    global mode
    while (not flagZone_interrupt):
        if((mode is not "RC")):
            for event in pygame.event.get():
                #if key is pressed move motor based on key            
                if kp.getKey('1'):
                    mode = "RC"
                    
                elif kp.getKey('2'):
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
    time.sleep(1.5)
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

    #update current position to the new position
    x1 = x2
    y1 = y2
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
    ser.write(b"Open Servo_grab\n")

def openGrabbingClaw():
    ser.write(b"Close Servo_grab\n")

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
  
    #Perform filter operation by anding frame using green_mas k
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
        if (cv2.contourArea(c) > 500):
            #Find the center of the box
            #TODO: YOU CAN REMOVE MOMENTS IF AVG IF EXT TOP AND BOT is GOOD
            extTop = tuple(c[c[:, :, 1].argmin()][0])        
            extBot = tuple(c[c[:, :, 1].argmax()][0])        

            avgX = (extTop[0]+extBot[0])/2

            return avgX

def centerWithFlag(Fx):
    global withinTol
    kP = .525 #0.625
    screen_center_x = 320 #center pixel of screen
    #print("NEWIMAGE")
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
            #print("err: ",err)
            if(err < 100):
                kP = 0.8
            elif(err < 60):
                kP = 1
            #print("kP: ", kP)
            Rspeed = kP*err
            print("AcutalSpeed :", Rspeed)
            if Rspeed > 80:#150
                Rspeed = 80
            elif Rspeed < 55 and Rspeed > 30:
                Rspeed = 55
                
                
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
        #if no flag found  then rotate 20 degrees and repeat main 
        RobotMotion.CCW(100)
        return False

def resetSubStates():
    global hallwayCenterGoing
    global flagZone
    global centered_w_Flag
    global captured
    global hallwayCenterComing
    global Delivered
    global flagZone_interrupt

    hallwayCenterGoing = False
    flagZone = False
    centered_w_Flag = False
    captured = False
    hallwayCenterComing = False
    Delivered = False
    flagZone_interrupt = False

def xSecsPassed(currTime, x):
    newTime = time.perf_counter()
    if((newTime - currTime) > x):
        return True
    else:
        return False

# RC mode funciton
def RCMODE():
    global mode
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
    #Mode Marvel Mind
            elif kp.getKey('m'):
                print(get_Position())
                
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
        global RobotMotion
        
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
        global flagZone_interrupt

        #Number of Flags delivered
        global FlagsDelivered

        #Timing Variable
        global NowTime

        #global current Position variables
        global x1, y1


        ser.write(b"Confirmation Mode\n")
        time.sleep(0.25)
        openGrabbingClaw()
        time.sleep(1)
        liftClaw()
        print("I am in Confirmation Mode :)")

        #NowTime = time.perf_counter()
        #x1, y1 = get_Position()
        x1=0
        counter = 0
        while(mode == "Confirmation"):
            counter +=1
            if(Avoiding and not blocked and mode == "Confirmation"):
                if(counter > 300):
                    #Buffer before you stop for some reason
                    time.sleep(1.5)
                    RobotMotion.stop()
                    time.sleep(5)
                    #x2, y2 = get_Position()
                    #UpdateHeading(x1,y1,x2,y2,15,y1) #orient yourself forwards towards corridor
                    counter = 0                    
                elif(x1 > 14.5):
                    Avoiding = False
                    FindingFlag = True
                else:
                    RobotMotion.forward(300)

            elif(FindingFlag and mode == "Confirmation"):
                
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
                
                #Stop the threads so that the processor can identify the flag
                flagZone_interrupt = True

                #I think flagZone_interrupt is always (flagZone and !captured)

                #Ready the claw
                lowerClaw()
                time.sleep(0.5)
                openGrabbingClaw()
                time.sleep(0.5)
                
                #when in goal zone find the flag:
                while (not centered_w_Flag and mode == "Confirmation"): 
                    imageFrame = captureImage(camera) #capture image
                    newX = getFlagCenter(imageFrame) # obtain flag center 
                    centered_w_Flag = centerWithFlag(newX) #center robot with the flag
                
                #Once Centered with the Flag, reset the interrupt and restart the threads
                flagZone_interrupt = False
                startThreads()
                
                #Capture the flag once identified
                curTime = time.perf_counter()
                while (not captured and mode == "Confirmation"):
                    if(time.perf_counter()-curTime < 4):
                        time.sleep(1.5)
                        RobotMotion.forward(150)
                    else:
                        time.sleep(1.5)
                        RobotMotion.stop()
                        #If robot moves for 2 seconds and has not captured then reorient
                        flagZone_interrupt = True 
                        centered_w_Flag = False
                        while (not centered_w_Flag and mode == "Confirmation"):
                            imageFrame = captureImage(camera) #capture image
                            newX = getFlagCenter(imageFrame) # obtain flag center 
                            centered_w_Flag = centerWithFlag(newX) #center robot with the flag
                            flagZone_interrupt = not centered_w_Flag
                            if(flagZone_interrupt == False):
                                startThreads()

                        curTime = time.perf_counter()
                    print(FL,FR)  
                    if((FL <= 15 or FR <= 15) and mode == "Confirmation"): #TODO Tune this number
                        time.sleep(1.5)
                        RobotMotion.stop()
                        captured = True
                        closeGrabbingClaw()
                        time.sleep(1)
                        liftClaw()
                        time.sleep(1)
                        FindingFlag = False
                        GettingFlag = True
                        
            
            elif(GettingFlag and mode == "Confirmation"):

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
        while(mode == "Scout"): pass
            
            
            
        
if __name__ == "__main__":

    #Create a camera object and initialize it
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1) # make buffer size = 1 

    #Make sure marvelmind is connected after connecting the motor controller
    #since we use ACM1 and not ACM0
    #Changes maxvaluesCount which is buffer size to 1
    #changed marvelmind address to 18
    #maxvaluescount=1
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1_MarvelMind", adr= 18, maxvaluescount=1, debug=False) # create MarvelmindHedge thread

    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread

    #Starts the ultrasonic and mode switch threads
    startThreads()

    while True:
        main()
        