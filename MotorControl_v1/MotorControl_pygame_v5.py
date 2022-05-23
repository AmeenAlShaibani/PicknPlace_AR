#! /usr/bin/python
import sys
from open_motor_serial import open_motor
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
from RobotMotion import RobotMotion

#############################
#baudRate = 115200
#port = "/dev/ttyACM0_teensy"
################################

#ARDUINO SERIAL INIT.
#######################################################################
ser = serial.Serial('/dev/ttyACM0_Arduino', 9600, timeout=1)
ser.reset_input_buffer()
#######################################################################

#import keyboard
sys.path.append("../src/open_motor/")


RobotMotion = RobotMotion()

# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

###############################################3
#comms = open_motor()
#comms.init_serial_port(port,baudRate,0.5)
################################################

#Initialize Robot Mode
mode = "RC"

#Initialize pygame keyboard
kp.init()

#initialize motor pwm vars

#############################################
#topright = 0
#topleft = 0
#bottomright = 0
#bottomleft = 0
#speed = 300
#########################################

#Create even for when US sensor reads something
US_event = Event()

############################################################################
#Motor Control function
#def MotorRun(Mot0, Mot1, Mot2, Mot3):
#    topright = Mot1*speed
#    topleft = Mot2*speed
#    bottomright = Mot0*speed
#    bottomleft = Mot3*speed

    #Adjusting motor direction
#    bottomright = -bottomright
#    topleft = -topleft
#    bottomleft = -bottomleft
    
    #Send the signals to 
#    comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)
##################################################################################3


# Function thread to get Ultrasonic data from Arduino 
def get_USDATA(outQueue):
    #TODO: Would there be a race condition between the .wait(), the Set(), and clear()?
    #  clear the event at beggining of thread 
    US_event.clear()

    while True:
        #If there are incoming bits 
        if  ser.inWaiting()>0: 
            #read the data which is a string in the form: US1,US2,US3,US4
            dataRaw = str(ser.readline())
            #Separate into a list with "," as separator
            dataList = dataRaw.split(",")

            #US1 = dataList[0]
            #US2 = dataList[1]
            #US3 = dataList[2]
            #US4 = dataList[3]

            #Only put data in queue if we are in confirmation or scout mode
            if(mode == "Confirmation" or mode == "Scout"):
                outQueue.put(dataList)
                ser.flushInput() #remove data after reading

                #Wait for the main thread to handle the US data before getting new data
                US_event.wait()

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
    RobotMotion.stopRobot()
    print("OG POS: ", x1, y1)

    #Get New position
    time.sleep(5)
    x2 = hedge.position()[1]
    y2 = hedge.position()[2]
    hedge.dataEvent.wait(1)
    hedge.dataEvent.clear()
    print("New POS: ", x2, y2)

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
    print("I need to turn: ", angTurn, " degrees to get to target")
    print("I need to turn clockwise for: ", turningTime, "s to align myself with goal")

    #TODO: You dont need any of this. Its here only for debugging and testing
    FullRotationTime = 9
    turningTime = angTurn * FullRotationTime/360
    PPR = angTurn * 1425.1/360 # this is the PPR value that we need to add/sub to each motor

    direc = np.cross(A,B)
    print(direc)
    
    #sleep buffer
    #TODO: See if you can reduce the time.sleep time. Maybe 0.025 or something like that is enough
    time.sleep(1)

    #Rotate as needed 
    RobotMotion.rotateDegrees(angTurn, direc)

    return x2, y2

#Function to find heading if you already have your position 1 and position 2 coordinates
def UpdateHeading(x1, y1, x2, y2, goalX, goalY, timeSleepForward=2):

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
    
    print("I need to turn: ", angTurn, " degrees to get to target")
    print("I need to turn clockwise for: ", turningTime, "s to align myself with goal")

    direc = np.cross(A,B)
    print(direc)
    
    #sleep buffer
    time.sleep(1)
    
    if(direc >= 0):
     #Turn clockwise
        print("turning CW")
        RobotMotion.CW()
        
    else:
        #turn CCW
        print("CCW")
        RobotMotion.CCW()

    time.sleep(turningTime)
    RobotMotion.stopRobot()
    print("Stopped")

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
                RobotMotion.stopRobot()
            
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
    
    #PiCamera
                
            # take a picture    
            elif kp.getKey('5'):
                camera.capture("/home/pi/Pictures/PresImage.jpg")
            
            #take a 5 sec video
            elif kp.getKey('6'):
                file_name = "/home/pi/Pictures/video_" + str(time.time()) + ".h264"
                print("Start recording...")
                camera.start_recording(file_name)
                
            elif kp.getKey('7'):
                camera.stop_recording()
                print("Done Recording")
                
    # Mode Switch Check
            elif kp.getKey('2'):
                mode = "Confirmation"
                
            elif kp.getKey('3'):
                mode = "Scout"
                
            #If key is lifted stop motors
        elif event.type == pygame.KEYUP:
            RobotMotion.stopRobot()
    
def main():
    global mode

    modeSwitch()

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
        ser.write(b"Confirmation Mode\n")
        #time.sleep(5)
        ser.write(b"Close Servo_grab\n")
        print("I am in Confirmation Mode :)")

        while(mode == "Confirmation"):

            #TODO: Handle based on respective values of US 
        
            # Right now, Arduino only sends when distance is 15 cm which might be too small 
            # you can increase the distance the arduino sends and then based on that distance 
            # handle robot behavior in here
            ##  For eg. you might not have anything in 15 cm but if you need to move forward 
            ##  ou would need a lot more space in front to be able to find your heading with the MM
            ##  Thus, you might need to see up to 200cm in front of you and if there is nothing within
            ##  200 cm you can move forward, but if anything is closer you would need to rotate again
            ##  and if you read something like 10cm then its too close etc. 
            ###   Additionally, this may be too fast in a for loop, that it would read empty 
            ###   ven tho there is still an object, since thread is not fast enough
            while (not US_q.empty()):
                US1, US2, US3, US4 = US_q.get()
                RobotMotion.rotateDegrees(90)
                US_event.set()
                time.sleep(0.05)

            #Sleep to stabalize MM
            time.sleep(2)

            #find initial heading and save the final position you end up at
            x1, y1 = FindHeading(3.4,3.3)

            #If there is an obstacle infront of you then turn right and move 
            if (not US_q.empty()):
                US1, US2, US3, US4 = US_q.get()
                RobotMotion.rotateDegrees(90)
                US_event.set()
                time.sleep(0.05)

            #if we turn 90 and there is still an obstacle then turn in the other direction
            if (not US_q.empty()):
                US1, US2, US3, US4 = US_q.get()
                RobotMotion.rotateDegrees(180, -1)
                US_event.set()
            
            #stabalize marvel mind and get pos
            time.sleep(2)
            x1 = hedge.position()[1]
            y1 = hedge.position()[2]
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()

            #Move forward
            RobotMotion.forward()
            time.sleep(3)
            RobotMotion.stopRobot()

            #Update the heading
            #TODO: it might be possible to calculate my position based on the amount i am sleeping
            # and the speed of the motor, since i already know my initial position and heading
            #But it may not be accurate
            time.sleep(5)
            x2 = hedge.position()[1]
            y2 = hedge.position()[2]
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()

            UpdateHeading(x1,y1, x2, y2, 3.4,3.3)

            #Keep doing above until you are within a tolerance of the goal (3.4,3.3)

            #Once you are there, then do the same kind of loop for the goal zone (4.6 , 4.5)

            #Once you are within a tolerance for goal zone, stop

            #Take a pic with the camera, and see if you can see flag 
            # if not : rotate 30~45 degrees and do the same 
            # if yes: continue 
           
            # get Cx from the image 
            # find distance between Cx and the center of the camera
            # rotate(Very slowely) so that Cx is in the center of the camera
            # maybe we can experimentally measure how long it takes to move X pixels 
            # if not then rotate slowly in the correct direction, and check if center is within tolerance
            # if center is within tolerance, then move forward, until you get a signal from the US 
            # signal will likely be >15 cm, so just move forward based on experimental amount 
            # then grab the flag
            
            # Then you can repeat the steps above, but aim to go for the center and the goal zone instead 




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
    camera = PiCamera()
    #TODO: you can remove camera flip, only there for debugging
    camera.vflip = True
    time.sleep(2)

    #Make sure marvelmind is connected after connecting the motor controller
    #since we use ACM1 and not ACM0
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1_MarvelMind", adr=None, debug=False) # create MarvelmindHedge thread

    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread
    
    #Create a Queue for Ultrasonic Data
    US_q = Queue()
    #Create US thread with the get_USDATA function and the corresponding queue
    US_thread = Thread(target=get_USDATA, args=(US_q, ), daemon=True)
    #Start US thread
    US_thread.start()

    while True:
        main()
        