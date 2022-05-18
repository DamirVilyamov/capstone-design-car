import cv2
import numpy as np
from filters import Filters
from motion import MotorControl
from time import sleep

capture = cv2.VideoCapture(0) 
capture.set(3, 320) # horizontal pixels 
capture.set(4, 240) # Vertical pixels

motion = MotorControl()

pixelsCountThreshold = 900
forwardPixelsCountThreshold = 800
filters = Filters()

while True:
    # ret : frame capture result 
    # frame : captured frame 
    ret, frame = capture.read()
    h, w, c = frame.shape

    if (ret):
        cv2.imshow('frame', frame)

        #blackWhiteImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('blackWhiteImage', blackWhiteImage)
        
        hsv = filters.applyHSV(frame)
        cv2.imshow('hsv', hsv)
        
        # edges = filters.applyCanny(hsv)

        # erosion_kernel = np.ones((5,5), np.uint8)

        # eroded_edges = filters.applyErosion(edges)
        # cv2.imshow('eroded edges', eroded_edges)

        # cv2.imshow('dilated edges', dilated_edges)

        roi = filters.getBottomROI(hsv)
        roi = filters.applyDilation(roi)
        cv2.imshow('roi', roi)

        leftLaneROI = filter.getRoi(frame, xStart = 0, xEnd = 0.5, yStart = 0.5, yEnd = 1.0)
        leftSidePixelsCount = cv2.countNonZero(leftLaneROI)
        rightLaneROI = filter.getRoi(frame, xStart = 0.5, xEnd = 1.0, yStart = 0.5, yEnd = 1.0)
        rightSidePixelsCount = cv2.countNonZero(leftLaneROI)

        sidesDifference = leftSidePixelsCount - rightSidePixelsCount

        sidesDifference*

        #right wheels speed
        #left wheels speed

        #delta speed = left wheels speed - right wheels speed

        # if deltaWidth = 0 , then right wheels speed = left wheels speed, then sidesDifference = deltaSpeed

        #if deltaWidth is minus sign, then deltaSpeed is minus, then right wheels speed is more

        #lets say width is 14 and 14 accodingly, and speed is 1.0 and 1.0, then k*deltaDistance = 

        

        median = (leftSidePixelsCount + rightSidePixelsCount) / 2

        koefficient = leftSidePixelsCount / median

        #multiply on the side where I want to turn
        
        if(leftSidePixelsCount < rightSidePixelsCount):
            rightSpeed = koefficient * 1.0
            leftSpeed = 1.0
            #count the number of pixels and give the power according to pixels
            #turn right

        if(rightSidePixelsCount < leftSidePixelsCount):
            leftSpeed = koefficient * 1.0
            rightSpeed = 1.0
            #count the number of pixels and give the power according to pixels
            #turn left


        if cv2.waitKey(1) > 0:
            break

capture.release() 
cv2.destroyAllWindows()