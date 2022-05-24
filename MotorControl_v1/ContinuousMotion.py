from ast import While
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


#import keyboard
sys.path.append("../src/open_motor/")
#Initialize Robot Motion class which controls robots
RobotMotion = RobotMotion()

#Initialize pygame keyboard
kp.init()
def main():
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

if __name__ == "__main__":
    while True:
        main()