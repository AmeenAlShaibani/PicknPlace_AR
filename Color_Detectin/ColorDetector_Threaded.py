# Python code for Multiple Color Detection
  
  
import numpy as np
import cv2
from CountsPerSec import CountsPerSec
from VideoGet import VideoGet
from VideoShow import VideoShow
  

def detectFlag(source=0):
    """
    This program has a dedicated thread for grabbing and showing images
    The main thread performs the color detection algorithm
    """

    video_getter = VideoGet(source).start()
    video_shower = VideoShow(video_getter.frame).start()

    while True:
        if video_getter.stopped or video_shower.stopped:
            video_shower.stop()
            video_getter.stop()
            break

 #####################Grab Image######################################   
    imageFrame = video_getter.frame
    
 ##################### Perform Mutation ######################################   

    #Blur the camera to reduce the noise
    for i in range(7):
        imageFrame = cv2.GaussianBlur(imageFrame,(5,5),0)

    # Convert the imageFrame in 
    # BGR(RGB color space) to 
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for green color and 
    # define mask
    green_lower = np.array([45, 55, 0], np.uint8)
    green_upper = np.array([120, 150, 70], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")
      
    # Apply a dilate filter to consolidate green pixels together
    green_mask - cv2.erode(green_mask, kernal, iterations =2)
    green_mask = cv2.dilate(green_mask, kernal, iterations = 4)
  
    #Perform filter operation by anding frame using green_mask
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask = green_mask)
     
  
    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    
    
    #Only Draw the Largest Countour
    if len(contours) != 0:
        # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)  
        imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                   (x + w, y + h),
                                   (0, 255, 0), 2)
        
        #Find the center of the box 
        M = cv2.moments(c)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        
        cv2.circle(imageFrame,(cx,cy),7,(255,255,255),-1)           
        cv2.putText(imageFrame, "Green Colour", (cx-20,cy-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    1.0, (255,255,255))
   
   
 ##################### Show image Mutation ######################################   
    video_shower.frame = imageFrame
    video_shower.frame = green_mask



def main():
    detectFlag(args["source"])

if __name__ == "__main__":
    main()