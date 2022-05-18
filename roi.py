import cv2
import numpy as np
from filters import Filters
from traffic_lights_detector import TrafficLightsDetector

filename = 'filename.avi'
capture = cv2.VideoCapture(0) 
capture.set(16, 1024) # horizontal pixels 
capture.set(9, 576) # Vertical pixels
# capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# capture.set(cv2.CAP_PROP_FRAME_HEIGH, 480)

white1 = np.array([0, 40, 40])
white2 = np.array([179, 255, 255])

pixelsCountThreshold = 300
filters = Filters()
trafficLightsDetector = TrafficLightsDetector()
while True:
    # ret : frame capture result 
    # frame : captured frame 
    ret, frame = capture.read()
    
    if (ret):
        h, w, c = frame.shape
        cv2.imshow('frame', frame)

        #blackWhiteImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('blackWhiteImage', blackWhiteImage)
        
        hsv = filters.applyHSV(frame)
        cv2.imshow('hsv', hsv)
        isRedTrafficLightInFrame = trafficLightsDetector.isRedTrafficLightInFrame(frame)
        
        # edges = filters.applyCanny(hsv)

        # erosion_kernel = np.ones((5,5), np.uint8)

        # eroded_edges = filters.applyErosion(edges)
        # cv2.imshow('eroded edges', eroded_edges)

        # dilated_edges = filters.applyDilation(eroded_edges)
        # cv2.imshow('dilated edges', dilated_edges)

        # roi = filters.getBottomROI(dilated_edges)
        # cv2.imshow('roi', roi)

        roi = filters.getBottomROI(hsv)
        roi = filters.applyDilation(roi)

        roi_h, roi_w = roi.shape

        #detect lines for forward
        leftLineDetector = roi[0:int(roi_h * 0.3), 0:int(roi_w * 0.3)]
        leftCnt = cv2.countNonZero(leftLineDetector)
        leftForwardLineVisible = leftCnt > pixelsCountThreshold

        rightLineDetector = roi[0:int(roi_h * 0.3), int(roi_w * 0.7):int(roi_w * 1.0)]
        rightCnt = cv2.countNonZero(rightLineDetector)
        leftForwardLineVisible = rightCnt > pixelsCountThreshold
        #------------------------

        #Detect left curve to turn right
        turnRightLineDetector = roi[int(roi_h * 0.9):int(roi_h * 1.0), int(roi_w * 0.3):int(roi_w * 0.45)]
        turnRightCnt = cv2.countNonZero(turnRightLineDetector)
        canTurnRight = turnRightCnt > pixelsCountThreshold

        #Detect right curve to turn left
        turnLeftLineDetector = roi[int(roi_h * 0.9):int(roi_h * 1.0), int(roi_w * 0.6):int(roi_w * 0.75)]
        turnLeftCnt = cv2.countNonZero(turnLeftLineDetector)
        canTurnLeft = turnLeftCnt > pixelsCountThreshold

        print('right forward count: ', rightCnt, 'left forward count: ', leftCnt)
        print('turn left count:', turnLeftCnt, 'turn right count:', turnRightCnt)
        print('turn left', canTurnLeft, 'turn right:', canTurnRight)
    
        cv2.imshow('roi', roi)
        # cv2.imshow('edge', edge)
        cv2.imshow('left line', leftLineDetector)
        cv2.imshow('right line', rightLineDetector)
        cv2.imshow('right curve', turnLeftLineDetector)
        cv2.imshow('left curve', turnRightLineDetector)

        if cv2.waitKey(1) > 0:
            break
    else: 
        capture.release()
        capture = cv2.VideoCapture(0) 
        print('CANNOT READ FRAME')

capture.release() 
cv2.destroyAllWindows()