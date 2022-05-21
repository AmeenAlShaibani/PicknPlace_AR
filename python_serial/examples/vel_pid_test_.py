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

def main():
    while True:

        comms.send_vel_goal(200,200,200,200)
        print("Response:" + comms.get_response())
        time.sleep(0.5)


        comms.send_vel_goal(0,0,0,0)
        print("Response:" + comms.get_response())
        time.sleep(0.5)

#         comms.send_vel_goal(-100,-100,-100,-100)
#         print("Response:" + comms.get_response())
#         time.sleep(0.5)
# 
#         comms.send_vel_goal(0,0,0,0)
#         print("Response:" + comms.get_response())
#         time.sleep(0.5)



        


if __name__ == "__main__":
    comms.send_pid_vars_solo_pos(0,1,0,0)
    comms.send_pid_vars_solo_pos(1,1,0,0)
    comms.send_pid_vars_solo_pos(2,0.5,1,0)
    comms.send_pid_vars_solo_pos(3,1,0,0)
    main()

