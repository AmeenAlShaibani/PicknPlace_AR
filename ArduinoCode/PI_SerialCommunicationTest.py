import serial #imports serial library for communication
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) # this is the name
    # '/dev/ttyACM0' : this is the name of the Arduino on the PI
    # 9600 is the baud rate specified in the arduino code
    # the timeout is in seconds. It means after 1 second, the pi will
        #output the bytes it has recieved, it will stop waiting for
        #more bytes to show up
    ser.reset_input_buffer()
    # This will flush any byte that could already be in the input buffer at that point
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip() #read and decode data
            print(line)