from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


class Vision(object):
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (320, 320)
        self.camera.framerate = 60
        self.rawCapture = PiRGBArray(self.camera, size=(320, 320))
        time.sleep(0.1)

    def circleDetection(self):
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            output = image.copy()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray,  
                cv2.HOUGH_GRADIENT, 1, 20, param1 = 100, 
                param2 = 30, minRadius = 10, maxRadius = 100) 
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for (x, y, r) in circles:
                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                    print("Circle found at: ", x, y, r)
            cv2.imshow("output", np.hstack([image, output]))
            key = cv2.waitKey(1) & 0xFF
            self.rawCapture.truncate(0)
            if key == ord("q"):
                break

    def getCirclesFromCam(self, debug=False):
        self.camera.capture(self.rawCapture, format="bgr")
        image = self.rawCapture.array
        output = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray,  
                cv2.HOUGH_GRADIENT, 1, 20, param1 = 100, 
                param2 = 30, minRadius = 10, maxRadius = 100) 
        circle_poses = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                if debug == True:
                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                cv2.imshow("output", np.hstack([image, output]))
                cv2.waitKey(0)
                x = (x-160)*(200/320)
                y = (y-160)*(200/320)+35
                circle_poses.append([x,y])
        self.rawCapture.truncate(0)
        return circle_poses

if __name__ == "__main__":
    v = Vision()
    while True:
        print(v.getCirclesFromCam(debug=True))