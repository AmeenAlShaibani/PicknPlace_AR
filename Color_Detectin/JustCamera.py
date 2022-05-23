import numpy as np
import cv2
from CountsPerSec import CountsPerSec 

##To read only an image for cv2 
def showCamera():
    while True:
        webcam = cv2.VideoCapture(0)
        _, imageFrame = webcam.read()
        cv2.imshow("live-feed",imageFrame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
                webcam.release()
                cv2.destroyAllWindows()
                break

showCamera()