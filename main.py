import cv2
import numpy as np
from filters import Filters
from motion import MotorControl
from time import sleep
from traffic_lights_detector import TrafficLightsDetector
from obstacle_detector import ObstacleDetector


filename = 'filename.avi'
capture = cv2.VideoCapture(0) 
capture.set(3, 320) # horizontal pixels 
capture.set(4, 240) # Vertical pixels

motion = MotorControl()

pixelsCountThreshold = 500
forwardPixelsCountThreshold = 1000
filters = Filters()
trafficLightsDetector = TrafficLightsDetector()
obstacleDetector = ObstacleDetector()

while True:
    # ret : frame capture result 
    # frame : captured frame 
    ret, frame = capture.read()
    h, w, c = frame.shape
    count = 0

    if (ret):
        cv2.imshow('frame', frame)

        #blackWhiteImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('blackWhiteImage', blackWhiteImage)
        
        hsv = filters.applyHSV(frame)
        cv2.imshow('hsv', hsv)
        isRedTrafficLightInFrame = trafficLightsDetector.isRedTrafficLightInFrame(frame)
        # isObstacle = obstacleDetector.isObstacleInRange(25)

        # edges = filters.applyCanny(hsv)

        # erosion_kernel = np.ones((5,5), np.uint8)

        # eroded_edges = filters.applyErosion(edges)
        # cv2.imshow('eroded edges', eroded_edges)

        # cv2.imshow('dilated edges', dilated_edges)

        roi = filters.getBottomROI(hsv)
        roi = filters.applyDilation(roi)
        cv2.imshow('roi', roi)

        roi_h, roi_w = roi.shape

        #detect lines for forward
        leftLineDetector = roi[0:int(roi_h * 0.5), 0:int(roi_w * 0.3)]
        leftCnt = cv2.countNonZero(leftLineDetector)
        leftForwardLineVisible = leftCnt > pixelsCountThreshold

        rightLineDetector = roi[0:int(roi_h * 0.5), int(roi_w * 0.7):int(roi_w * 1.0)]
        rightCnt = cv2.countNonZero(rightLineDetector)
        leftForwardLineVisible = rightCnt > pixelsCountThreshold
        #------------------------

        #Detect left curve to turn right
        turnRightLineDetector = roi[int(roi_h * 0.9):int(roi_h * 1.0), int(roi_w * 0.2):int(roi_w * 0.35)]
        turnRightCnt = cv2.countNonZero(turnRightLineDetector)
        canTurnRight = turnRightCnt > pixelsCountThreshold

        #Detect right curve to turn left
        turnLeftLineDetector = roi[int(roi_h * 0.9):int(roi_h * 1.0), int(roi_w * 0.7):int(roi_w * 0.85)]
        turnLeftCnt = cv2.countNonZero(turnLeftLineDetector)
        canTurnLeft = turnLeftCnt > pixelsCountThreshold

        # #if (rightCnt <= forwardPixelsCountThreshold or leftCnt <= forwardPixelsCountThreshold):
        print('RED LIGHT', isRedTrafficLightInFrame)
        # if(isRedTrafficLightInFrame or isObstacle):
        #         print('TRUE TRUE')
        #         motion.stop()
        # else:
        # print('FALSE')
        #motion.forward(power=0.6)
        #sleep(2)
        #if(count == 0):
          #  if(isRedTrafficLightInFrame):
         #       count++
           #     motion.stop()
            
        #if (isRedTrafficLightInFrame):
         #   motion.stop()

        if (canTurnRight):
            #turnRight
            #motion.stop()
            #sleep(0.2)
            motion.right(power=0.7)
            #sleep(0.2)
            
        if (canTurnLeft):
            #turnLeft
            #motion.stop()
            #sleep(0.2)
            motion.left(power=0.9)
            #sleep(0.2)

        if (rightCnt >= forwardPixelsCountThreshold and leftCnt >= forwardPixelsCountThreshold and not canTurnRight and not canTurnLeft):
            #move forvard
            motion.forward(power=0.7)    

        print('right forward count: ', rightCnt, 'left forward count: ', leftCnt)
        print('turn left count:', turnLeftCnt, 'turn right count:', turnRightCnt)
        print('turn left', canTurnLeft, 'turn right:', canTurnRight)
        
        # cv2.imshow('edge', edge)
        cv2.imshow('left line', leftLineDetector)
        cv2.imshow('right line', rightLineDetector)
        cv2.imshow('right curve', turnLeftLineDetector)
        cv2.imshow('left curve', turnRightLineDetector)

        if cv2.waitKey(1) > 0:
            break

capture.release() 
cv2.destroyAllWindows()