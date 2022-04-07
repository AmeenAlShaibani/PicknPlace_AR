                                                                              #! /usr/bin/python3

baudRate = 115200
port = "/dev/ttyACM0"

import sys
#import keyboard
sys.path.append("../src/open_motor/")
# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

from open_motor_serial import open_motor
import KeyPressModule as kp
import time
import pygame
from MarvelMind import MarvelmindHedge

comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

#Initialize pygame keyboard
kp.init()

#initlize motor pwm vars
topright = 0
topleft = 0
bottomright = 0
bottomleft = 0

def MotorRun(Mot0, Mot1, Mot2, Mot3):
    speed = 400 #CHANGE SPEED HERE
    topright = Mot1*speed
    topleft = Mot2*speed
    bottomright = Mot0*speed
    bottomleft = Mot3*speed

    #CHANGE THIS IF MOTORS ARE TURNING OPPOSITE WAY
    bottomright = -bottomright
    topleft = -topleft
    bottomleft = -bottomleft


    #time.sleep(0.1)
    #comms.send_pwm_goal(0,0,0,0)
    #time.sleep(0.1)
    comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)
    #print("Response:" + comms.get_response())

#LEFT SIDE IS NEGATIVE

def main():
#     #THIS FUNCTION IS USED TO PLAN PATH USING MARVEL MIND
#     hedge.dataEvent.wait(1)
#     hedge.dataEvent.clear()
#     if (hedge.positionUpdated):
#          hedge.print_position()
         #         if(hedge.position()[1] >5.88
#            and hedge.position()[1] < 6.15
#            and hedge.position()[2] > 2.10
#            and hedge.position()[2] < 2.5):
#             print("Within Box")
#             comms.send_pwm_goal(0,0,0,0)
#             hedge.stop()
#             sys.exit()

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
                MotorRun(-1,-1,1,1)

            #CW rotate
            elif kp.getKey('a'):
                MotorRun(1,1,-1,-1)

            #Stop
            elif kp.getKey('s'):
                MotorRun(0,0,0,0)

        #If key is lifted stop motors
        if event.type == pygame.KEYUP:
            MotorRun(0,0,0,0)


if __name__ == "__main__":

    #Make sure marvelmind is connected after connecting the motor controller
    #since we use ACM1 and not ACM0
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1", adr=None, debug=False) # create MarvelmindHedge thread

    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]

    hedge.start() # start thread
    while True:
        main()