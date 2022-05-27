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
 
#ARDUINO SERIAL INIT.
#######################################################################
ser = serial.Serial('/dev/ttyACM0_Arduino', 57600, timeout=1)
ser.reset_input_buffer()
####################################################################### 
 
#import keyboard
sys.path.append("../src/open_motor/")

#Initialize Robot Motion class which controls robots
RobotMotion = RobotMotion()

# Function thread to get Ultrasonic data from Arduino 
def get_USDATA():
    global RobotMotion
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
            print(US1, US2, US3, US4)
            print("ultrasonic is far away: ", US4 > 15)
            #if((US1 > 15) and (US2 > 15) and (US3 > 15) and (US4 > 15) and lock.locked()):
                #lock.release()
                #print("released")
            print("lock is locked: ", RobotMotion.lock.locked())
            print("we should lock: ", US4 < 15 and (not RobotMotion.lock.locked()))
            if(US4 > 15 and RobotMotion.lock.locked()):
                RobotMotion.lock.release()
                print("released")
            elif(US4 < 15 and (not RobotMotion.lock.locked())):
                print("locked")
                RobotMotion.lock.acquire()
                RobotMotion.right()
                
def main():
    global RobotMotion
    #print("RUNNING MAIN")
    RobotMotion.forward()
        
if __name__ == "__main__":

    #Create US thread with the get_USDATA function and the corresponding queue
    US_thread = Thread(target=get_USDATA, args=())
    #Start US thread
    US_thread.start()
    while True:
        main()
        