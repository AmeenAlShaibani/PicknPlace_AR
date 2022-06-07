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
hallwayCenterComing = False
Delivered = False

US_interrupt = False # interrupt used to stop threads when needed
Mode_interrupt = False # interrupt used to stop threads when needed

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

def start_USThread():
    #Create US thread with the get_USDATA function
    US_thread = Thread(target=get_USDATA, args=(), daemon=True)
    #Start US thread
    US_thread.start()

#Starts the Ultrasonic and ModeSwitch threads:
def startThreads():
    #First empty out the arduino serial buffer
    ser.flushInput()

    #Create US thread with the get_USDATA function
    US_thread = Thread(target=get_USDATA, args=(), daemon=True)
    #Start US thread
    US_thread.start()

    #Create thread to change the mode of the robot
    mode_thread = Thread(target=modeSwitch, args=(), daemon=True)
    #Start Mode thread
    mode_thread.start()

def stopThreads():
    global US_interrupt
    global Mode_interrupt

    Mode_interrupt = True
    US_interrupt = True

# Function thread to get Ultrasonic data from Arduino 
def get_USDATA():
    global blocked
    global FL
    global FR
    global US_interrupt

    MovingRight = False
    MovingLeft = False
    
    #TODO: Would there be a race condition between the .wait(), the Set(), and clear()?
    while (not US_interrupt):#  clear the event at beggining of thread 
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
            print(SR, ER, FR, FL, EL, SL)
            #70 130 130 70
            #55 90 90 55

            #THIS EXECUTES AT TIMES WHEN IT SHOULDNT BE DOING SO
            if((ER < 140 or FR < 140 or FL < 140 or EL < 140) and (mode == "Confirmation") and Avoiding):
                blocked = True
                #Turn towards left or right based on which reads a further distance
                if((SR > SL) and not MovingLeft):
                    MovingRight = True
                    RobotMotion.right(200) # was 150
                elif(SR < SL and not MovingRight):
                    MovingLeft = True
                    RobotMotion.left(200)

            elif((ER > 140 and FR > 140 and FL > 140 and EL > 140) and (mode == "Confirmation")  and Avoiding):
                MovingRight = False
                MovingLeft = False
                blocked = False
            
            #return FR, FL # if there is a reading return front two
    
    #return None # if there is not reading return none
    
#Function to check the mode 
def modeSwitch():
    global mode
    global Mode_interrupt

    while (not Mode_interrupt):
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
    time.sleep(1.5)

    #Rotate as needed 
    RobotMotion.rotateDegrees(angTurn, -direc)

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
    #PPR = angTurn * 1425.1/360 # this is the PPR value that we need to add/sub to each motor
    
    #print("I need to turn: ", angTurn, " degrees to get to target")
    #print("I need to turn clockwise for: ", turningTime, "s to align myself with goal")

    direc = np.cross(A,B)
    print(direc)
    
    #sleep buffer
    time.sleep(1)
    
    if(direc >= 0):
     #Turn clockwise
       # print("turning CW")
        RobotMotion.CCW()
        
    else:
        #turn CCW
        #print("CCW")
        RobotMotion.CW()

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
        if (cv2.contourArea(c) > 1000):
            #Find the center of the box
            #TODO: YOU CAN REMOVE MOMENTS IF AVG IF EXT TOP AND BOT is GOOD
            
            #extTop = tuple(c[c[:, :, 1].argmin()][0])        
            #extBot = tuple(c[c[:, :, 1].argmax()][0])        

            #avgX = (extTop[0]+extBot[0])/2

            M = cv2.moments(c)
            cx = int(M["m10"]/M["m00"])

            return cx

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
            elif Rspeed < 55 and Rspeed > 20:
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

def reset_interrupts():
    global US_interrupt
    global Mode_interrupt

    US_interrupt = False
    Mode_interrupt = False
    
def set_interrupts():
    global US_interrupt
    global Mode_interrupt

    US_interrupt = True
    Mode_interrupt = True

def resetSubStates():
    global hallwayCenterGoing
    global flagZone
    global centered_w_Flag
    global captured
    global hallwayCenterComing
    global Delivered
    global US_interrupt
    global Mode_interrupt


    hallwayCenterGoing = False
    flagZone = False
    centered_w_Flag = False
    captured = False
    hallwayCenterComing = False
    Delivered = False
    US_interrupt = False
    Mode_interrupt = False

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
        global US_interrupt
        global Mode_interrupt

        #Number of Flags delivered
        global FlagsDelivered

        #Timing Variable
        global NowTime

        #global current Position variables
        global x1
        global y1


        ser.write(b"Confirmation Mode\n")
        time.sleep(0.25)
        openGrabbingClaw()
        time.sleep(1)
        liftClaw()
        print("I am in Confirmation Mode :)")

        #Starts the ultrasonic and mode switch threads
        startThreads()

        #NowTime = time.perf_counter()
        x1, y1 = get_Position() #Get initial Position
        counter = 0
        while(mode == "Confirmation"):
            counter +=1
            x1, y1 = get_Position()
            if(Avoiding and not blocked and mode == "Confirmation"):
                if(x1 > 14.5):
                    Avoiding = False
                    FindingFlag = True
                    US_interrupt = True # kill the ultrasonic thread after avoiding stuff                    
                elif(counter > 500):
                    time.sleep(1.5)
                    RobotMotion.stop()
                    time.sleep(1.5)
                    x2, y2 = get_Position()
                    UpdateHeading(x1,y1,x2,y2,15, y2)
                else:
                    RobotMotion.forward(300)

            elif(FindingFlag and mode == "Confirmation"):


                print("Going to center of hallway")
                time.sleep(0.5)
                RobotMotion.stop()
                x1, y1 = get_Position()
                if(not hallwayCenterGoing):
                    if(x1 > 13.53 and y1 < 1.4 and mode == "Confirmation"):
                            hallwayCenterGoing = True
                    else:
                        time.sleep(1.5)
                        x2, y2 = get_Position()
                        UpdateHeading(x1,y1,x2,y2,15.65, 0.82)
                        centerCounter = 0 
                        #Go to center of hallway
                        while(not hallwayCenterGoing and mode == "Confirmation"):
                            centerCounter += 1
                            x1, y1 = get_Position()
                            if(x1 > 13.53 and y1 < 1.4 and mode == "Confirmation"):
                                hallwayCenterGoing = True
                            elif(centerCounter > 500):
                                print("Updating Heading")
                                time.sleep(1.5)
                                RobotMotion.stop()
                                time.sleep(2)
                                x2,y2 = get_Position()
                                UpdateHeading(x1,y1,x2,y2,15.65, 0.82)
                            else:
                                RobotMotion.forward(300)
                
                print("Going to Flag Zone")
                #go to the Flag zone
                while(not flagZone and mode == "Confirmation"):
                    x1, y1 = get_Position()
                    if(x1 > 17.6 and mode == "Confirmation"):
                        flagZone = True
                    elif(mode == "Confirmation"):
                        FindHeading(18.4,1.1)
                        RobotMotion.forward(200)
                        time.sleep(4) #TODO: Tune this number
                        RobotMotion.stop()
                        time.sleep(1.5) #TODO Tune htis number
                
                #Stop the threads so that the processor can identify the flag
                stopThreads()

                #Ready the claw
                openGrabbingClaw()
                time.sleep(0.8)
                lowerClaw()
                time.sleep(0.8)
                
                print("Centering with flag")
                #when in goal zone find the flag:
                while (not centered_w_Flag and mode == "Confirmation"): 
                    imageFrame = captureImage(camera) #capture image
                    newX = getFlagCenter(imageFrame) # obtain flag center 
                    centered_w_Flag = centerWithFlag(newX) #center robot with the flag
                
                #Once Centered with the Flag, reset the interrupt and restart the threads
                reset_interrupts()
                startThreads()
                
                print("Capturing Flag")
                #Capture the flag once identified
                curTime = time.perf_counter()
                FlagDetectorCounter = 0
                Orienting = False
                while (not captured and mode == "Confirmation"):
                    if((FL <= 15 or FR <= 15) and mode == "Confirmation"): #TODO Tune this number
                        FlagDetectorCounter += 1
                        RobotMotion.stop()
                        if(FlagDetectorCounter == 4):
                            #Stop and stop US thread
                            time.sleep(1.5)
                            RobotMotion.stop()
                            captured = True
                            US_interrupt = True
                            #capture flag and set next state
                            closeGrabbingClaw()
                            time.sleep(1)
                            liftClaw()
                            time.sleep(1)
                            FindingFlag = False
                            GettingFlag = True
                            
                    elif((time.perf_counter() - curTime) > 6):
                        print("REORIENTING")
                        Orienting = True
                        time.sleep(0.5)
                        RobotMotion.stop()
                        #If robot moves for 2 seconds and has not captured then reorient
                        set_interrupts()
                        centered_w_Flag = False
                        while (not centered_w_Flag and mode == "Confirmation"):
                            imageFrame = captureImage(camera) #capture image
                            newX = getFlagCenter(imageFrame) # obtain flag center 
                            centered_w_Flag = centerWithFlag(newX) #center robot with the flag
                            if(centered_w_Flag):
                                # if we are center, then restart the threads
                                reset_interrupts()
                                startThreads()

                        curTime = time.perf_counter()
                        Orienting = False
                    elif(mode == "Confirmation" and not Orienting):
                        print("MOBING")
                        time.sleep(1.5)
                        RobotMotion.forward(200)
                    #print(FL,FR)  
                             
            
            elif(GettingFlag and mode == "Confirmation"):
                
                print("Going to HallWay center")
                RobotMotion.stop()
                time.sleep(2)
                x1, y1 = get_Position()
                x1, y1 = FindHeading(15.65, 0.82)
                while((not hallwayCenterComing) and mode == "Confirmation"):
                    if(x1 > 14.53 and y1 < 1 and mode == "Confirmation"):
                        hallwayCenterGoing = True
                    elif(mode == "Confirmation"):
                        time.sleep(1.5)
                        RobotMotion.forward(300)
                        time.sleep(4) #TODO: Tune this number
                        RobotMotion.stop()
                        x2, y2 = get_Position()
                        UpdateHeading(x1, y1, x2, y2, 15.65, 0.82)
                        
                        
                print("Going to the goal Zone")
                #To to goal Zone:
                while(not Delivered and mode == "Confirmation"):
                    x1, y1 = get_Position()
                    if(WithinTolerance(0.5,x1,y1,15.43,6.8) and mode == "Confirmation"):
                        #Drop the Flag
                        RobotMotion.stop()
                        lowerClaw()
                        time.sleep(2)
                        openGrabbingClaw()
                        time.sleep(1)
                        Delivered = True
                        FlagsDelivered += 1
                        
                        #if both flags are delivered then end
                        if(FlagsDelivered == 2 and mode == "Confirmation"):
                            mode = "RC"
                        #if only 1 flag then get the other flag
                        elif(mode == "Confirmation"):
                            RobotMotion.backward()
                            time.sleep(1)
                            RobotMotion.stop()
                            #If you havent found both flags, return to finding flag state
                            GettingFlag = False
                            FindingFlag = True
                            resetSubStates()
                            
                    elif(mode == "Confirmation"):
                        FindHeading(15.43,6.83)
                        RobotMotion.forward(200)
                        time.sleep(4)
                        RobotMotion.stop()
                        time.sleep(2)
            
            if(blocked and mode == "Confirmation" and (not flagZone)):
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
    time.sleep(1)
    liftClaw()
     
    while True:
        main()
    