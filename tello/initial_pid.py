from warnings import resetwarnings
from djitellopy import Tello
import cv2 as cv
import numpy as np
import threading
import rospy
import rosbag
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image



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
    
    def __init__(self):       
        self.tello = Tello()
        self.tello.connect()
        print(self.tello.get_battery())

        self.tello.send_rc_control(0,0,0,0)
        self.tello.takeoff()


        #PID Values
                #-0.00000000000025
        #yaw_controller = PID(0.7, -0.00000000025, 0.1)
        #yaw_controller.send(None)

        self.z_controller = PID(0.2, 0.0000000000000000000000005, 0.08)
        self.z_controller.send(None)

        self.x_controller = PID(0.08, 0.000000000000031, 0.05)
        self.x_controller.send(None)

        self.y_controller = PID(0.6, 0, 0.07)
        self.y_controller.send(None)
        
        self.z = 0
        self.y = 0
        self.x = 0
        
        self.tic = rospy.Time.now().to_sec()
        self.toc = rospy.Time.now().to_sec()
        self.dt = -1
        
        self.firstSight = True
        self.no_detection = 0

        self.bag = open(r'/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/tello_test.txt', 'w')
    
    def execute_PID(self, msg):
        
        debug=True
        
        info_list=list(map(int, msg.header.frame_id.split()))
        print('object {:d} {:d}'.format(info_list[0], info_list[1]))
        self.bag.write('object {:d} {:d} {:f}\n'.format(info_list[0], info_list[1], rospy.Time.now().to_sec()))

        if info_list!=[0,0,0]:
            self.no_detection=0
            if self.firstSight:
                self.tic=rospy.Time.now().to_sec()
                self.firstSight = False
                
            else:
                center_offset = (320-info_list[0], 240-info_list[1])
                
                self.toc=rospy.Time.now().to_sec()
                self.dt= self.toc-self.tic
                print('dt = ', self.dt)
                self.tic=self.toc
                self.x = -1 * int(self.x_controller.send((self.dt, center_offset[0])))
                self.y = -1*int(self.y_controller.send((self.dt, info_list[2])))
                self.z = int(self.z_controller.send((self.dt, center_offset[1])))
                
                print('speed commands {:f} {:f} {:f}\n'.format(self.x, self.y, self.z))
                self.bag.write('speed commands {:f} {:f} {:f} {:f}\n'.format(self.x, self.y, self.zrospy.Time.now().to_sec()))
            
 
                if not debug: self.tello.send_rc_control(x, y, z, 0)
                
        else:
            self.no_detection+=1
            if self.no_detection>10:     
                self.x_step=0.0
                self.y_step=0.0
                self.z_step=0.0
            if self.no_detection>40:
                self.firstSight=True
                    
            

        
  
def main():

    # initiate node
    rospy.init_node('wingman_node', anonymous=True)

    
    #reset pid controllers; zero the I
    global reset
    reset = True
    
    # controller object
    wm = wingman()
    print('!!!! TELLO INIT !!!!')
    
    reset=False
    
    # ROS loop rate
    rate = rospy.Rate(20.0)
    
    #subscribe to yolo center offset and object's distance away
    rospy.Subscriber('detections', Image, wm.execute_PID)
    print("!!!! TELLO SUB / PUB INIT !!!!!")
    
    
    count=0

    while not rospy.is_shutdown():
        rospy.spin()
            
    print('shutting down tello_pid')
    wm.bag.close()
    wm.tello.send_rc_control(0,0,0,0)
    wm.tello.end()


    
if __name__ == '__main__':
    try:
        main()   
    except rospy.ROSInterruptException:
        pass  
