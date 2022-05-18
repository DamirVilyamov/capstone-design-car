import cv2
import numpy as np
from filters import Filters
from motion import MotorControl
from time import sleep

capture = cv2.VideoCapture(0) 
capture.set(3, 320) # horizontal pixels 
capture.set(4, 240) # Vertical pixels

motion = MotorControl()
filters = Filters()

while True:
    # ret : frame capture result 
    # frame : captured frame 
    ret, frame = capture.read()
    h, w, c = frame.shape

    if (ret):
        cv2.imshow('frame', frame)


        if cv2.waitKey(1) > 0:
            break
    else:
        print('CANNOT retrieve frame')

capture.release() 
cv2.destroyAllWindows()