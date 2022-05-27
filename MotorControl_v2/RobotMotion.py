from open_motor_serial import open_motor
import time
import threading

class RobotMotion:
    
    ## Class Initializer
    def __init__(self):
        self.comms = open_motor()
        self.comms.init_serial_port("/dev/ttyACM0_teensy",115200,0.5)
        self.lock = threading.Lock()
        
# FIXME: this is completely wrong, you shouldnt have speed, this is position 
    def PosRun(self, Mot0, Mot1, Mot2, Mot3, speed=300):
        #TODO: I need to the get the current PPR values for each motor, and add the PPR value I need 
        toprightWheel = Mot1*speed
        topleftWheel = Mot2*speed
        bottomrightWheel = Mot0*speed
        bottomleftWheel = Mot3*speed

        #Adjusting motor direction
        bottomrightWheel = -bottomrightWheel
        topleftWheel = -topleftWheel
        bottomleftWheel = -bottomleftWheel
        
        #Send the signals to 
        self.comms.send_pos_goal(bottomrightWheel,toprightWheel,topleftWheel,bottomleftWheel)

    #Motor Control function
    def MotorRun(self, Mot0, Mot1, Mot2, Mot3, speed=300):
        

        toprightWheel = Mot1*speed
        topleftWheel = Mot2*speed
        bottomrightWheel = Mot0*speed
        bottomleftWheel = Mot3*speed

        #Adjusting motor direction
        bottomrightWheel = -bottomrightWheel
        topleftWheel = -topleftWheel
        bottomleftWheel = -bottomleftWheel
        
        #Send the signals to 
        self.comms.send_pwm_goal(bottomrightWheel,toprightWheel,topleftWheel,bottomleftWheel)

    #Rotate in place by indicating degrees and direction.
    #  direction >= 0 ------> CW
    #  direction < 0  ------> CCW
    def rotateDegrees(self, degrees, direction=0):
        FullRotationTime = 9
        turningTime = degrees * FullRotationTime/360
    
        if(direction >= 0):
        #Turn clockwise
            self.CW()
            
        else:
            #turn CCW
            self.CCW()

        time.sleep(turningTime)
        self.stopRobot()

    def forward(self, speed=300):
        self.MotorRun(1,1,1,1,speed)

    def backward(self, speed=300):
        self.MotorRun(-1,-1,-1,-1,speed)

    def right(self, speed=300):
        self.MotorRun(1,-1,1,-1,speed)

    def left(self, speed=300):
        self.MotorRun(-1,1,-1,1,speed)

    def topright(self, speed=300):
        self.MotorRun(1,0,1,0,speed)

    def topleft(self, speed=300):
        self.MotorRun(0,1,0,1,speed)

    def bottomright(self, speed=300):
        self.MotorRun(-1,0,-1,0,speed)

    def bottomleft(self, speed=300):
        self.MotorRun(0,-1,0,-1,speed)

    def CCW(self, speed=150):
        self.MotorRun(1,1,-1,-1,speed)#we are turning at 150

    def CW(self, speed=150):
        self.MotorRun(-1,-1,1,1,speed)

    def stopRobot(self):
        self.MotorRun(0,0,0,0)
