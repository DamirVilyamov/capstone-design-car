import cv2
import numpy as np
from filters import Filters

minimum_red_pixels_count = 600

class TrafficLightsDetector:
    def __init__(self):
        self.filters = Filters()
        self.lower_red_boundary = np.array([0, 40, 40])
        self.higher_red_boundary = np.array([10, 255, 255])

    def isRedTrafficLightInFrame(self, frame):
        roi = self.filters.getRoi(frame, yStart = 0, yEnd = 0.2, xStart = 0, xEnd = 0.2)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv = cv2.inRange(hsv, self.lower_red_boundary, self.higher_red_boundary)
        redPixelsCount = cv2.countNonZero(hsv)
        print('red pixels count ', redPixelsCount)
        return redPixelsCount > minimum_red_pixels_count

