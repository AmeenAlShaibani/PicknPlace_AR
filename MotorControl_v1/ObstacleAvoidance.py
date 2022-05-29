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

blocked = True

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
            SR = float(dataList[0])
            ER = float(dataList[1])
            FR = float(dataList[2])
            FL = float(dataList[3])
            EL = float(dataList[4])   
            SL = float(dataList[5])   
   
            #print(US1, US2, US3, US4)

            if(ER < 55 or FR < 100 or FL < 100 or EL < 55 ):
                blocked = True
                #Turn towards left or right based on which reads a further distance
                if(SR > SL):
                    RobotMotion.right()
                else:
                    RobotMotion.left()

            elif(ER > 55 and FR > 100 and FL > 100 and 55 > 55 ):
                blocked = False
            
            return FR, FL # if there is a reading return front two
        
        return None # if there is not reading return none

def main():
    
    global blocked
    while(not blocked):#print("RUNNING MAIN")
        RobotMotion.forward()
        
if __name__ == "__main__":

    #Create US thread with the get_USDATA function and the corresponding queue
    US_thread = Thread(target=get_USDATA, args=())
    #Start US thread
    US_thread.start()
    while True:
        main()
        