#! /usr/bin/python3

baudRate = 115200
port = "/dev/ttyACM0_teensy"

import sys
sys.path.append("../src/open_motor/")
# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

from open_motor_serial import open_motor
import time

comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

def updateEncVal():
    motorDict = comms.get_response_json()
    pos0 = motorDict["pos0"]
    pos1 = motorDict["pos1"]
    pos2 = motorDict["pos2"]
    pos3 = motorDict["pos3"]
    
    return pos0, pos1, pos2, pos3

def main():
    test_val = 0
    pos0, pos1, pos2, pos3 = updateEncVal()
    #comms.send_pos_goal(pos0+10000,pos1-10000,pos2+10000,pos3+10000)
    #comms.send_pos_goal(0000,-0000,0000,0000)
    #comms.send_pwm_goal(00,00,00,0)
    #comms.send_pos_goal(00,00,00,0)

#     comms.send_pwm_goal(0,0,0,0)
#     comms.set_gear_ratio(0,1425.1)
#     comms.set_gear_ratio(1,1425.1)
#     comms.set_gear_ratio(2,1425.1)
#     comms.set_gear_ratio(3,1425.1)
    
#     comms.send_pid_vars_solo_pos(0,0.5,0,0)
#     comms.send_pid_vars_solo_pos(1,0.5,0,0)
#     comms.send_pid_vars_solo_pos(2,0.5,0,0)
#     comms.send_pid_vars_solo_pos(3,0.5,0,0)


    #print(pos0)
#     fullRot = 1465*6
#     ROT = 1425.1
#     
#     forward5ft = 1425.1*5
#     sideways3ft = 1425.1 * 3
#     sideways4ft = 1425.1 * 4
# 
#     forward4ft = 1425.1 * 5
#     
#     pos0, pos1, pos2, pos3 = updateEncVal()
#     print(pos0,pos1,pos2,pos3)
#     #Forwards 5 ft
#     comms.send_pos_goal(pos0-forward5ft,pos1+forward5ft,pos2-forward5ft,pos3-forward5ft)
#     time.sleep(7)

#     pos0, pos1, pos2, pos3 = updateEncVal()
#     print(pos0,pos1,pos2,pos3)
#     #right 3 ft
#     comms.send_pos_goal(pos0+sideways3ft,pos1+sideways3ft,pos2+sideways3ft,pos3-sideways3ft)
#     time.sleep(7)
#     pos0, pos1, pos2, pos3 = updateEncVal()
#     print(pos0,pos1,pos2,pos3)
#         
#     pos0, pos1, pos2, pos3 = updateEncVal()
#     print(pos0,pos1,pos2,pos3)
# #     #Forwards 5 ft
#     comms.send_pos_goal(pos0-forward4ft,pos1+forward4ft,pos2-forward4ft,pos3-forward4ft)
#     time.sleep(7)
#     
#     pos0, pos1, pos2, pos3 = updateEncVal()
#     print(pos0,pos1,pos2,pos3)
#     #right 3 ft
#     comms.send_pos_goal(pos0-sideways4ft,pos1-sideways4ft,pos2-sideways4ft,pos3+sideways4ft)
#     time.sleep(7)
#     
#     
#     pos0, pos1, pos2, pos3 = updateEncVal()
#     print(pos0,pos1,pos2,pos3)
# #     #Forwards 5 ft
#     comms.send_pos_goal(pos0-forward4ft,pos1+forward4ft,pos2-forward4ft,pos3-forward4ft)
#     time.sleep(7)
# 
# 
#     pos0, pos1, pos2, pos3 = updateEncVal()
#     print(pos0,pos1,pos2,pos3)
# #    #left 3 ft
#     comms.send_pos_goal(pos0+sideways3ft,pos1+sideways3ft,pos2+sideways3ft,pos3-sideways3ft)
#     time.sleep(10)
    
    
    
#    comms.send_pos_goal(0,0,0,0)
#     comms.send_pos_goal(fullRot,fullRot,fullRot,fullRot)
#     comms.send_pos_goal(ROT,ROT,ROT,ROT)

    

    while True:

        
        comms.send_vel_goal(0,0,0,0)
        print("Response:" + comms.get_response())
        print("Stopping")
        time.sleep(1)

        comms.send_vel_goal(200,2000,200,200)
        print("Response:" + comms.get_response())
        print("GOING UP")
        time.sleep(5)

if __name__ == "__main__":
#     comms.send_pid_vars_solo_vel(0,1,0,0)
#     comms.send_pid_vars_solo_vel(1,1,0,0)
#     comms.send_pid_vars_solo_vel(2,0.5,1,0)
#     comms.send_pid_vars_solo_vel(3,1,0,0)
   while True:
    main()


