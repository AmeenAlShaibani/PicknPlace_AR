from json.decoder import JSONDecoder
import serial
import json
import time

class open_motor:

    ## Class Initializer
    def __init__(self):
        self.ser = serial.Serial()
        self.msg_data = {}
        self._port = None
        self._baudrate = None
        self._timeout = None


    ## Intializes and opens Serial Port
    def init_serial_port(self, port, baudRate, timeout):
        self._port = port
        self._baudrate = baudRate
        self._timeout = timeout
        self.ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout)

    ## Check if Serial port is open
    def isOpen(self):
       return self.ser.is_open
    
    ## Close Serial Port
    def close_port(self):
        self.ser.close()
        return self.ser.closed

    ## Get response from serial port as a string
    def get_response(self):
        self.ser.flushInput()
        data = self.ser.readline().decode("utf-8")
        return data

    ## Return Response from Serial port as a JSON dictionary
    def get_response_json(self):
        data = self.get_response()
        json_data = json.loads(data)
        return json_data

    ## Send a PWM duty cycle goal to all 4 motors
    def send_pwm_goal(self,pwm0,pwm1,pwm2,pwm3):
        self.msg_data["command"] = "pwm_direct"
        self.msg_data["pwm0"] = pwm0
        self.msg_data["pwm1"] = pwm1
        self.msg_data["pwm2"] = pwm2
        self.msg_data["pwm3"] = pwm3

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Send a Velocity PID goal to all 4 motors
    def send_vel_goal(self,vel0,vel1,vel2,vel3):
        self.msg_data["command"] = "vel_pid"
        self.msg_data["vel0"] = vel0
        self.msg_data["vel1"] = vel1
        self.msg_data["vel2"] = vel2
        self.msg_data["vel3"] = vel3

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Send a Position PID goal to all 4 motors
    def send_pos_goal(self,pos0,pos1,pos2,pos3):
        self.msg_data["command"] = "pos_pid"
        self.msg_data["pos0"] = pos0
        self.msg_data["pos1"] = pos1
        self.msg_data["pos2"] = pos2
        self.msg_data["pos3"] = pos3

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Send same PID Vars for velocity to all motors
    def send_pid_vars_vel_all(self, P, I, D):
        self.msg_data["command"] = "pid_vars_vel_all"
        self.msg_data["P"] = P
        self.msg_data["I"] = I
        self.msg_data["D"] = D

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)
    
    ## Send same PID Vars for position to all motors
    def send_pid_vars_pos_all(self, P, I, D):
        self.msg_data["command"] = "pid_vars_pos_all"
        self.msg_data["P"] = P
        self.msg_data["I"] = I
        self.msg_data["D"] = D

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Send PID variables for velocity to individual motor
    def send_pid_vars_solo_vel(self, mot_num, P, I, D):
        self.msg_data["command"] = "pid_vars_solo_vel"
        self.msg_data["mot_num"] = mot_num
        self.msg_data["P"] = P
        self.msg_data["I"] = I
        self.msg_data["D"] = D

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Send PID variables for position to individual motor
    def send_pid_vars_solo_pos(self, mot_num, P, I, D):
        self.msg_data["command"] = "pid_vars_solo_pos"
        self.msg_data["mot_num"] = mot_num
        self.msg_data["P"] = P
        self.msg_data["I"] = I
        self.msg_data["D"] = D

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Set Motor Direction to individual motor, true and false are used arbitrarily 
    def set_motor_direction(self, mot_num, dir):
        self.msg_data["command"] = "set_dir"
        self.msg_data["mot_num"] = mot_num
        if dir == True:
            self.msg_data["dir"] = 1
        elif dir == False:
            self.msg_data["dir"] = 0

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Set Gear Ratio for a specific motor using encoder tickers per rotation
    def set_gear_ratio(self, mot_num, ticks_per_rotation):
        self.msg_data["command"] = "set_tpr"
        self.msg_data["mot_num"] = mot_num
        self.msg_data["tpr"] = ticks_per_rotation

        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    ## Set Gear Ration for all motors using encoder ticks per rotation
    def set_gear_ratio_all(self, mot_num, ticks_per_rotation):
        self.msg_data["command"] = "set_tpr"
        self.msg_data["mot_num"] = mot_num
        self.msg_data["tpr"] = ticks_per_rotation
        
        msg = json.dumps(self.msg_data).encode("ascii")
        self.ser.write(msg)

    

   

    






