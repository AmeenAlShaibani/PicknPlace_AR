from RobotMotion import RobotMotion
from open_motor_serial import open_motor
import numpy as np
import cv2
from threading import Thread


##To read only an image for cv2 
webcam = cv2.VideoCapture(0)

def showCamera():
    while True:
        _, imageFrame = webcam.read()
        imageFrame = cv2.flip(imageFrame,0)
        cv2.imshow("live-feed",imageFrame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
                webcam.release()
                cv2.destroyAllWindows()
                break

def main():
    while True:
        flag = input("r or l")
        if (flag == "l"):
            RobotMotion.rotateDegrees(25)
        elif(flag=="r"):
            RobotMotion.rotateDegrees(25,-1)
        else:
            break


if __name__ == "__main__":
    RobotMotion = RobotMotion()
    cameraThread = Thread(target=showCamera, args=(), daemon=False)
    cameraThread.start()
    main()