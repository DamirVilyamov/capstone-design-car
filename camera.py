import cv2
import sys
import numpy as np
from motion import MotorControl
from time import sleep

motion = MotorControl()


capture = cv2.VideoCapture(0) 
capture.set(3, 320) # horizontal pixels 
capture.set(4, 240) # Vertical pixels
white1 = np.array([0, 0, 150])
white2 = np.array([179, 255, 255])
ret, frame = capture.read()

while True:
    # ret : frame capture result 
    # frame : captured frame 
    ret, frame = capture.read()
    h, w, c = frame.shape   

    if (ret):
    # convert image to Grayscale
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img, (5,5), 0)
        edge = cv2.Canny(blur, 50, 150)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # hsv = cv2.inRange(hsv, white1, white2)
        roi = edge[int(h*0.7):int(h), 0:w]

        #print(roi.shape)
        roi_h, roi_w = roi.shape
        # print(roi_h)
        # print(roi_w)

        leftLineDetector = roi[0:int(roi_h * 0.7), 0:int(roi_w * 0.3)]
        leftCnt = cv2.countNonZero(leftLineDetector)

        rightLineDetector = roi[0:int(roi_h * 0.7), int(roi_w * 0.75):int(roi_w * 1.0)]
        rightCnt = cv2.countNonZero(rightLineDetector)

        turnRightLineDetector = roi[int(roi_h * 0.3):int(roi_h * 0.6), int(roi_w * 0.3):int(roi_w * 0.4)]
        turnRightCnt = cv2.countNonZero(turnRightLineDetector)

        turnLeftLineDetector = roi[int(roi_h * 0.3):int(roi_h * 0.6), int(roi_w * 0.6):int(roi_w * 0.7)]
        turnLeftCnt = cv2.countNonZero(turnLeftLineDetector)

        cv2.imshow('frame', frame)
        cv2.imshow('roi', roi)
        # print('rightCnt:', rightCnt, 'leftCnt:', leftCnt)

        if (rightCnt <= 50 or leftCnt <= 50):
            print('leftCnt:', leftCnt, 'turnRightCnt:', turnRightCnt)
            print('rightCnt:', rightCnt, 'turnLeftCnt', turnLeftCnt)
            if (turnRightCnt >= 50 and leftCnt >= 50):
                #turnRight
                motion.smoothRight(power=0.8)
                sleep(0.2)
                
            if (turnLeftCnt >= 50 and rightCnt >= 50):
                #turnLeft
                motion.smoothLeft(power=0.8)
                sleep(0.2)

        if (rightCnt >= 50 and leftCnt >= 50):
            #move forvard
            motion.forward(power=0.5)        


        # print("Left count", leftCnt)

        # print("Right count", rightCnt)

        # cv2.imshow('edge', edge)
        # cv2.imshow('hsv', hsv)
        cv2.imshow('left', leftLineDetector)
        cv2.imshow('right', rightLineDetector)
        cv2.imshow('turnLeft', turnLeftLineDetector)
        cv2.imshow('turnRight', turnRightLineDetector)


        if cv2.waitKey(1) > 0:
            motion.stop()
            break

motion.stop()
capture.release() 
cv2.destroyAllWindows()