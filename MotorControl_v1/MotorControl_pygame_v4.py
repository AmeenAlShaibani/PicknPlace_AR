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

baudRate = 115200
port = "/dev/ttyACM0_teensy"

#ARDUINO SERIAL INIT.
#######################################################################
ser = serial.Serial('/dev/ttyACM0_Arduino', 9600, timeout=1)
ser.reset_input_buffer()
#######################################################################

#import keyboard
sys.path.append("../src/open_motor/")

# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

#Initialize Robot Mode
mode = "RC"

#Initialize pygame keyboard
kp.init()

#initlize motor pwm vars
topright = 0
topleft = 0
bottomright = 0
bottomleft = 0

speed = 300

Avoided = False
PikedUP = False

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

def updateEncVal():
    motorDict = comms.get_response_json()
    pos0 = motorDict["pos0"]
    pos1 = motorDict["pos1"]
    pos2 = motorDict["pos2"]
    pos3 = motorDict["pos3"]   
    return pos0, pos1, pos2, pos3


#This function gives the motors a position goal to get to. Works off of PPR
def PosRun(Mot0, Mot1, Mot2, Mot3):
    #TODO: I need to the get the current PPR values for each motor, and add the PPR value I need 
    topright = Mot1*speed
    topleft = Mot2*speed
    bottomright = Mot0*speed
    bottomleft = Mot3*speed

    #Adjusting motor direction
    bottomright = -bottomright
    topleft = -topleft
    bottomleft = -bottomleft
    
    #Send the signals to 
    comms.send_pos_goal(bottomright,topright,topleft,bottomleft)

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
                
def SlowNearPoint(x,y):
    global speed
    xPos = hedge.position()[1];
    yPos = hedge.position()[2];
    DistanceToTarget = ((x-xPos)**2 +(y-yPos)**2)**0.5;
    #print(DistanceToTarget)
    if(DistanceToTarget < 1.5):
        speed = 200
    
    else:
        speed = 400

def SlowNearAllPoints(PathNum):
    if (PathNum == 1):
        SlowNearPoint(3.6,4.6)
    elif (PathNum == 2):
        SlowNearPoint(3.8,1.5)
    elif (PathNum == 3):
        SlowNearPoint(3.6,4.6)
        
# def SlowNearPointV2(p):
#     global speed
#     xPos = hedge.position()[1];
#     yPos = hedge.position()[2];
#     DistanceToTarget = ((x-xPos)**2+(y-yPos))**0.5;
#     if(DistanceToTarget < 0.3):
#         speed = 100
#     
#     else:
#         speed = 400

def FindHeadingDOT(goalX, goalY, timeSleepForward=2):
    time.sleep(5)
    #Get Original Position
    x1 = hedge.position()[1];
    y1 = hedge.position()[2];
    hedge.dataEvent.wait(1)
    hedge.dataEvent.clear()
    
    #Make it run for a little bit
    MotorRun(1,1,1,1)
    time.sleep(timeSleepForward)
    MotorRun(0,0,0,0)
    print("OG POS: ", x1, y1)

    #Get New position
    time.sleep(5)
    x2 = hedge.position()[1];
    y2 = hedge.position()[2];
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
    
    FullRotationTime = 9
    turningTime = angTurn * FullRotationTime/360
    PPR = angTurn * 1425.1/360 # this is the PPR value that we need to add/sub to each motor
    
    #for some reason this doesnt run hahahahahahahahahahahahah
    print("I neeed to turn: ", angTurn, " degrees to get to target")
    print("I need to turn clockwise for: ", turningTime, "s to align myslef with goal")

    direc = np.cross(A,B)
    print(direc)
    
    #sleep buffer
    time.sleep(1)
    
    if(direc >= 0):
     #Turn clockwise
        print("turning CW")
        MotorRun(0.5,0.5,-0.5,-0.5)
        
    else:
        #turn CCW
        print("CCW")
        MotorRun(-0.5,-0.5,0.5,0.5)

    time.sleep(turningTime)
    print("Stopped")
    MotorRun(0,0,0,0)
    
def main():
    global mode
    global Avoided
    global PikedUP
    
    modeSwitch()

             #LEVEL 1
####################################################################
    if mode == "RC":
        ser.write(b"RC Mode\n")
        print("I am in RC Mode :)")
        while(mode == "RC"):
            
            for event in pygame.event.get():
                #if key is pressed move motor based on key
                if event.type == pygame.KEYDOWN:
                    
                    if kp.getKey('UP'):
                       MotorRun(1,1,1,1)

                        #Backward
                    elif kp.getKey('DOWN'):
                        MotorRun(-1,-1,-1,-1)

                        #Right
                    elif kp.getKey('RIGHT'):
                        MotorRun(1,-1,1,-1)

                        #Left
                    elif kp.getKey('LEFT'):
                        MotorRun(-1,1,-1,1)

                    #topright
                    elif kp.getKey('e'): #SEE IF YOU CAN DO 'forward' AND 'RIGHT'
                        MotorRun(1,0,1,0)

                    #topleft
                    elif kp.getKey('q'): #SEE IF YOU CAN DO 'LEFT' AND 'RIGHT'
                        MotorRun(0,1,0,1)

                    #bottomright
                    elif kp.getKey('c'): #SEE IF YOU CAN DO 'forward' AND 'RIGHT'
                        MotorRun(-1,0,-1,0)

                    #bottomleft
                    elif kp.getKey('z'): #SEE IF YOU CAN DO 'LEFT' AND 'RIGHT'
                        MotorRun(0,-1,0,-1)
                        
                    #CCW rotate
                    elif kp.getKey('d'):
                        MotorRun(0.5,0.5,-0.5,-0.5)#we are turning at 150

                    #CW rotate
                    elif kp.getKey('a'):
                        MotorRun(-0.5,-0.5,0.5,0.5)

                    #Stop
                    elif kp.getKey('s'):
                        MotorRun(0,0,0,0)
                    
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
                    MotorRun(0,0,0,0)
                    
                    
    #LEVEL 2
####################################################################
    elif mode == "Confirmation":
        #ser.write(b"Confirmation Mode\n")
        #time.sleep(5)
        ser.write(b"Close Servo_grab\n")
        grabbedFlag = False
        PathNum = 0
        print("I am in Confirmation Mode :)")
        
        fullRot = 1465*6
        ROT = 1425.1
        
        forward5ft = 1425.1*5
        sideways3ft = 1425.1 * 3
        sideways4ft = 1425.1 * 4

        forward4ft = 1425.1 * 5
        
#         pos0, pos1, pos2, pos3 = updateEncVal()
#         print(pos0,pos1,pos2,pos3)
#         #Forwards 5 ft
#         comms.send_pos_goal(pos0-forward5ft,pos1+forward5ft,pos2-forward5ft,pos3-forward5ft)
#         time.sleep(7)
# 
#         pos0, pos1, pos2, pos3 = updateEncVal()
#         print(pos0,pos1,pos2,pos3)
#         #right 3 ft
#         comms.send_pos_goal(pos0+sideways3ft,pos1+sideways3ft+4000,pos2+sideways3ft,pos3-sideways3ft-4000)
#         time.sleep(7)
#         pos0, pos1, pos2, pos3 = updateEncVal()
#         print(pos0,pos1,pos2,pos3)
#             
#         pos0, pos1, pos2, pos3 = updateEncVal()
#         print(pos0,pos1,pos2,pos3)
#     #     #Forwards 5 ft
#         comms.send_pos_goal(pos0-forward4ft,pos1+forward4ft,pos2-forward4ft,pos3-forward4ft)
#         time.sleep(7)
#         
#         pos0, pos1, pos2, pos3 = updateEncVal()
#         print(pos0,pos1,pos2,pos3)
#         #right 3 ft
#         comms.send_pos_goal(pos0-sideways4ft,pos1-sideways4ft+3000,pos2-sideways4ft,pos3+sideways4ft-3000)
#         time.sleep(7)
    
        time.sleep(2)
        xPos = hedge.position()[1];
        yPos = hedge.position()[2];
        
        hedge.dataEvent.wait(1)
        hedge.dataEvent.clear()
        
        if(xPos > 4.1 and xPos < 4.6
            and yPos > 4.1 and yPos < 4.6):
                PikedUP = True
        
        if Avoided == False:
            xPos = hedge.position()[1];
            yPos = hedge.position()[2];
            
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()
            
            if(xPos > 2 and xPos < 3.5
            and yPos > 2.5 and yPos < 4.5):
                Avoided = True
                
                
            FindHeadingDOT(3.4,3.3)
        
        elif PikedUP == False:
            FindHeadingDOT(4.6,4.5,1)
        
        else:
            MotorRun(0,0,0,0)
            ser.write(b"Open Servo_grab\n")
            time.sleep(1)
            ser.write(b"Open Servo_lift\n")
            sys.exit()
        
               
            
            
            #LEVEL 3
####################################################################
    elif mode == "Scout":
        ser.write(b"Scout Mode\n")
        print("I am in Scout Mode :)")
        while(mode == "Scout"):
            
            #Check if user switched mode        
            modeSwitch()
            
        
if __name__ == "__main__":

    #Make sure marvelmind is connected after connecting the motor controller
    #since we use ACM1 and not ACM0
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1_MarvelMind", adr=None, debug=False) # create MarvelmindHedge thread

    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread
    
    #Create a camera object and initialize it
    camera = PiCamera()
    camera.vflip = True
    time.sleep(2)
    

    while True:
        main()
        