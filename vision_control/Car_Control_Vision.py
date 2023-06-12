import cv2
import numpy as np
from LaneDetectionModule import getLaneCurve
from MotorModule import Control
import KeyboardModule as KP
import utlis


motor = Control(25, 24, 23, 22, 17, 27)
intialTrackBarVals = [131, 164, 95, 222 ]
utlis.initializeTrackbars(intialTrackBarVals)
KP.init()


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    frameCounter = 0
    while True:
        frameCounter += 1
        if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0
 
        success, img = cap.read()
        img = cv2.resize(img,(480,240))
        curve = getLaneCurve(img,display=2)
        print(curve)
        if KP.getKey('DOWN'):
            motor.Drive(0,0)
        elif KP.getKey('UP'):
            if (curve == 1):
                motor.Drive(0.35,0.8)
            elif (curve == -1):
                motor.Drive(0.35,-0.8)
            else:
                motor.Drive(0.35,0)
        cv2.waitKey(1)
