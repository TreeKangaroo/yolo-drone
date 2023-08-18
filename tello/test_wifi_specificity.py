#!/usr/bin/env python3

import time
from djitellopy import tello
import cv2
import logging

drone1 = tello.Tello('172.17.0.2', state_port=9890, video_port=21111, control_port=8809)
#drone2 = tello.Tello('192.168.10.1', state_port=8890, video_port=11111)
drone1.connect()
#drone2.connect()
print(drone1.get_battery())
#print(drone2.get_battery())

drone1.streamon()
#drone2.streamon()


while True:
    img1 = drone1.get_frame_read().frame
    img1 = cv2.resize(img1, (360, 240))
    
    #img2 = drone2.get_frame_read().frame
    #img2 = cv2.resize(img2, (360, 240))
    
    cv2.imshow("drone 1 results", img1)
    #cv2.imshow("drone 2 results", img2)
    cv2.waitKey(1)

