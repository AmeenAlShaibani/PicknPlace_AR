#! /usr/bin/python
import sys
from open_motor_serial import open_motor
import serial
import time
import numpy as np
from threading import Thread, Lock
from RobotMotion import RobotMotion




#You should make a thread and a lock
# you start the thread after sending a signal to the arduino in the confirmaiton mode
# the ConMode does confirmation ideally* basically
# The thread handles the case when US is read.
# if it is close enough, you lock and move away
# when you get to the flag zone checkpoint, update a global state variable b locking in main thread
# in flag zone checkpoint, the US handles action based on distance differently since it assumes frotn sensors are the flag
 
#Issue: 

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
FindingFlag = False
GettingFlag = False

path1 = False
path2 = False
path3 = False
path4 = False 

blocked = False
Driving = False
######################

# Function thread to get Ultrasonic data from Arduino 
def get_USDATA(outQueue):
    #TODO: Would there be a race condition between the .wait(), the Set(), and clear()?
    #  clear the event at beggining of thread 
    while True:
        #If there are incoming bits 
        if  ser.inWaiting()>0: 
            #read the data which is a string in the form: US1,US2,US3,US4
            dataRaw = str(ser.readline())
            #Separate into a list with "," as separator
            dataList = dataRaw.split(",")

            Lock.acquire()
            RobotMotion.right()
            #US1 = dataList[0]
            #US2 = dataList[1]
            #US3 = dataList[2]
            #US4 = dataList[3]
        else:
            Lock.release()
                
def main():
    currentime = time.time()
    while((time.time() - currentime) > 10):
        RobotMotion.forward()
    RobotMotion.stopRobot()


            
        
if __name__ == "__main__":

    #Create US thread with the get_USDATA function and the corresponding queue
    US_thread = Thread(target=get_USDATA, args=(), daemon=True)
    lock = Lock()
    #Start US thread
    US_thread.start()

    main()
        