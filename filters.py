import cv2
import numpy as np

class Filters:
    def __init__(self):
        self.erosion_kernel = np.ones((5,5), np.uint8)
        self.lower_color_boundary = np.array([45, 40, 40])
        self.higher_color_boundary = np.array([75, 255, 255])
        self.lower_color_boundary_day = np.array([25, 40, 40])
        self.higher_color_boundary_day = np.array([45, 255, 255])

    def applyHSV(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.inRange(hsv, self.lower_color_boundary_day, self.higher_color_boundary_day)
        return hsv
    
    def getBottomROI(self, frame, ratio = 0.5):
        h, w = frame.shape
        roi = frame[int(h*ratio):int(h), 0:w]
        return roi

    def applyCanny(self, frame):
        edge = cv2.Canny(frame, 50, 150)
        return edge

    def applyDilation(self, frame):
        dilated_frame = cv2.dilate(frame, self.erosion_kernel, iterations = 1)
        return dilated_frame

    def applyErosion(self, frame):
        eroded_frame = cv2.erode(frame, self.erosion_kernel, iterations = 1)
        return eroded_frame
        
    def applyBlur(self, frame):
        blurredImage = cv2.GaussianBlur(frame, (5,5), 0)
        return blurredImage

    def getRoi(self, frame, xStart, yStart, xEnd, yEnd):
        h, w, c = frame.shape
        roi = frame[int(h*yStart):int(h*yEnd), int(w*xStart):int(w*xEnd)]
        return roi