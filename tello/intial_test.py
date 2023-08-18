#!/usr/bin/env python3

from djitellopy import tello
import cv2
import time

me = tello.Tello()
#cap = cv2.VideoCapture(0)
me.connect()
print(me.get_battery())
me.streamon()
me.takeoff()
while True:
    me.send_rc_control(0,0,0,0)



