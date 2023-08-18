#!/usr/bin/env python3

from djitellopy import Tello
import cv2
import numpy as np
import rospy
import rospkg
import time

class wingman:
    
    def PID(self, Kp, Ki, Kd, MV_bar=0):
        e_prev = 0
        #t_prev = -1
        I = 0
        MV = MV_bar
        while True:
            global reset
            if reset==True:
                I=0
            dt, e = yield MV
            #e = SP - PV
            P = Kp * e
            I = I + Ki*e*dt
            D = Kd*(e - e_prev)/dt
            MV = (MV_bar + P + I + D)*10**-3
            #print(f'MV:{int(MV)} P:{int(P)} I:{int(I)} D:{int(D)} e:{e} e_prev:{e_prev}')

            e_prev = e
            #t_prev = t
                
    def __init__(self, drone_number=0):
        
        if drone_number !=0:
            self.tello = Tello('172.17.0.2', state_port=9890, video_port=21111, control_port=8809)
        else:
            self.tello = Tello('192.168.10.1', state_port=8890, video_port=11111, control_port=8889)
        self.tello.connect()
        self.tello.streamon()
        print(self.tello.get_battery())

        self.tello.send_rc_control(0,0,0,0)
        
        global reset
        reset = True
        
        #these numbers actually work sort of
        self.z_controller = self.PID(250.0, 0.0000000000000000000005, 80.0)
        self.z_controller.send(None)

        self.x_controller = self.PID(250.0, 0.000000000031, 50.0)
        self.x_controller.send(None)
        #end working numbers

        self.y_controller = self.PID(600.0, 0.0, 70.0)
        self.y_controller.send(None)
    
        self.z = 0
        self.y = 0
        self.x = 0
        
        self.tic = rospy.Time.now().to_sec()
        self.toc = rospy.Time.now().to_sec()
        self.dt = -1
        
        self.firstSight = True
        self.no_detection = 0

        self.bag = open(r'/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/tello_trial1.txt', 'w')
        time.sleep(10)
        #self.tello.takeoff()
        #self.tello.move_up(100)
        self.PIDoverride = False

    def get_frame(self):
        img = self.tello.get_frame_read().frame
        img = cv2.resize(img, (416, 416))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return [img, rospy.Time.now().to_sec()]
    
    def location_cb(self, msg):
        
        self.PIDoverride = True
        info = str(msg.data).split()
      
        if info.count('right'):
            self.x = -30
        elif info.count('left'):
            self.x = 30
        
        if info.count('up'):
            self.z = -30
        elif info.count('down'):
            self.z = 30
   
        
    def execute_PID(self, info_list):

        debug=True
        
        #print('tello object {:d} {:d}'.format(info_list[0], info_list[1]))
        self.bag.write('object {:d} {:d} {:f}\n'.format(info_list[0], info_list[1], rospy.Time.now().to_sec()))

        if info_list[0:3]!=[0,0,0]:
            self.no_detection=0
            if self.firstSight:
                self.tic=rospy.Time.now().to_sec()
                self.firstSight = False
            
            elif self.PIDoverride:
                #print('PID OVERRIDDEN')
                print('tello speed commands {:f} {:f} {:f}\n'.format(self.x, self.y, self.z))
                self.bag.write('speed commands {:f} {:f} {:f} {:f}\n'.format(self.x, self.y, self.z, rospy.Time.now().to_sec()))
                if not debug: self.tello.send_rc_control(self.x, 0, self.z, 0)    
                self.PIDoverride = False
            else:
                center_offset = (208-info_list[0], 208-info_list[1])
                
                self.toc=rospy.Time.now().to_sec()
                self.dt= self.toc-info_list[3]
                #print('dt = ', self.dt)
                self.tic=self.toc
                self.x = -1 * int(self.x_controller.send((self.dt, center_offset[0])))
                #self.y = -1*int(self.y_controller.send((self.dt, info_list[2])))
                if abs(center_offset[0])<50 and abs(center_offset[1])<50:
                    self.y = 40
                else:
                    self.y=0
                self.z = int(self.z_controller.send((self.dt, center_offset[1])))
                
                #print('tello speed commands {:f} {:f} {:f}\n'.format(self.x, self.y, self.z))
                self.bag.write('speed commands {:f} {:f} {:f} {:f}\n'.format(self.x, self.y, self.z, rospy.Time.now().to_sec()))
            
 
                if not debug: self.tello.send_rc_control(self.x, self.y, self.z, 0)
                
        else:
            self.no_detection+=1
            if self.no_detection>10:     
                self.x_step=0.0
                self.y_step=0.0
                self.z_step=0.0
            if self.no_detection>40:
                self.firstSight=True
            if not debug: self.tello.send_rc_control(0, 0, 0, 0)


