#!/usr/bin/env python3

from djitellopy import tello
import cv2
import numpy as np

class tello_stream:
    def __init__(self):
        self.drone = tello.Tello()
        self.drone.connect()
        self.drone.streamon()


    def get_frame(self):
        img = self.drone.get_frame_read().frame
        img = cv2.resize(img, (416, 416))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #color_array = np.asanyarray(img)
        return img


