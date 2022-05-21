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

kp.init()

def MotorRun(int Mot0, intMot1, int Mot2, int Mot3)
        speed = 0
        topright = Mot1*speed
        topleft = Mot2*speed
        bottomright = Mot0*speed
        bottomleft = Mot3*speed

        bottomleft = -bottomleft
        topleft = -topleft

        #time.sleep(0.1)
        #comms.send_pwm_goal(0,0,0,0)
        #time.sleep(0.1)
    comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)
    print("Response:" + comms.get_response())

#LEFT SIDE IS NEGATIVE

def main():
    topright = 0
    topleft = 0
    bottomright = 0
    bottomleft = 0
    try:
        hedge.dataEvent.wait(1)
        hedge.dataEvent.clear()
        if (hedge.positionUpdated):
            #hedge.print_position()
            if(hedge.position()[1] >5.88
               and hedge.position()[1] < 6.15
               and hedge.position()[2] > 2.10
               and hedge.position()[2] < 2.5):
                print("Within Box")
                comms.send_pwm_goal(0,0,0,0)
                hedge.stop()
                sys.exit()
    except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()
        # Forward
    if kp.getKey('UP'):
        topright = 100
        topleft = 100
        bottomright = 100
        bottomleft = 100

        bottomleft = -bottomleft
        topleft = -topleft

        #time.sleep(0.1)
        #comms.send_pwm_goal(0,0,0,0)
        #time.sleep(0.1)
        comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)
        print("Response:" + comms.get_response())

        #Backward
    elif kp.getKey('DOWN'):
        topright = -100
        topleft = -100
        bottomright = -100
        bottomleft = -100

        bottomleft = -bottomleft
        topleft = -topleft

        #time.sleep(0.1)
        #comms.send_pwm_goal(0,0,0,0)
        #time.sleep(0.1)        comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)
        print("Response:" + comms.get_response())

        #Right
    elif kp.getKey('RIGHT'):
        topright = -speed
        topleft = speed
        bottomright = speed
        bottomleft = -speed

        bottomleft = -bottomleft
        topleft = -topleft

        #time.sleep(0.1)
        #comms.send_pwm_goal(0,0,0,0)
        #time.sleep(0.1)
        comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)
        print("Response:" + comms.get_response())

        #Left
    elif kp.getKey('LEFT'):
        topright = 200
        topleft = -200
        bottomright = -200
        bottomleft = 200

        time.sleep(0.1)
        comms.send_pwm_goal(0,0,0,0)
        time.sleep(0.1)
        comms.send_pwm_goal(bottomright,topright,topleft,bottomleft)
        print("Response:" + comms.get_response())

        #Stop
    elif kp.getKey('d'):
        comms.send_pwm_goal(0,0,0,0)
        print("Response:" + comms.get_response())

        #Rotate Left



if __name__ == "__main__":

    hedge = MarvelmindHedge(tty = "/dev/ttyACM1", adr=None, debug=False) # create MarvelmindHedge thread

    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]

    hedge.start() # start thread
    while True:
        main()