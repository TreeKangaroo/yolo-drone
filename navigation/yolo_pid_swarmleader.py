#!/usr/bin/env python
# ROS python API

import rospy
import rosbag
import numpy as np

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from std_msgs.msg import String, Float64

from mavros_msgs.srv import *
from utils.fcuModes import fcuModes
from sensor_msgs.msg import NavSatFix
import time 

class Controller:

    #PID control method
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
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('011111000111', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 8

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 0.0
        # update the setpoint message with the required altitude
        #self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        # initial values for setpoints
        self.sp.velocity.x = 0.0
        self.sp.velocity.y = 0.0
        
         #initialize pid controllers
        self.x_pid_cnt = self.PID(0.6, 0.000001, 0.07)
        self.x_pid_cnt.send(None)
        
        self.y_pid_cnt = self.PID(1.8, 0.000001, 0.21)
        self.y_pid_cnt.send(None)
        
        self.z_pid_cnt = self.PID(0.6, -0.000001, 0.08, MV_bar=-0.1)
        self.z_pid_cnt.send(None)
        
        # setup step size for x, y, z direction
        self.x_step = 0.0
        self.y_step = 0.0
        self.z_step = 0.0
        
        self.pre_sp_x = 0.0
        self.pre_sp_y = 0.0
        self.pre_sp_z = 0.0
        
        self.yaw_step = 0.0
        
        # manuver label
        self.manuver = 'landed'
        
        self.gps = NavSatFix()
        self.alt=0.00
        self.rel_alt=0.00
        
        self.tic = rospy.Time.now().to_sec()
        self.toc = rospy.Time.now().to_sec()
        self.dt = -1
        
        self.firstSight = True
        self.no_detection = 0
        self.dv_thresh = 0.1
        self.vmax =1.0
        
        #new rosbag for data logging
        self.bag = open(r'/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/trial1.txt', 'w')
        self.sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        #print('position {:f} {:f} {:f}'.format(self.local_pos.x, self.local_pos.y, self.local_pos.z))
        self.bag.write('position {:f} {:f} {:f}\n'.format(self.local_pos.x, self.local_pos.y, self.local_pos.z))
        #print('Current drone state: ', self.state.mode)

    
    #update setpoint velocities etc
    def updateSp(self):
        
        #check acceleration
        if (self.x_step - self.pre_sp_x) > self.dv_thresh:
            self.x_step = self.pre_sp_x + self.dv_thresh
        elif (self.x_step - self.pre_sp_x) < -self.dv_thresh:
            self.x_step = self.pre_sp_x - self.dv_thresh  
        
        #check velocity magnitude
        if self.x_step > self.vmax:
            self.sp.velocity.x = self.vmax
        elif self.x_step < -self.vmax:
            self.sp.velocity.x = -self.vmax
        else:
            self.sp.velocity.x = self.x_step
       
         #check acceleration
        if (self.y_step - self.pre_sp_y) > self.dv_thresh:
            self.y_step = self.pre_sp_y + self.dv_thresh
        elif (self.y_step - self.pre_sp_y) < -self.dv_thresh:
            self.y_step = self.pre_sp_y - self.dv_thresh  
        
        #check velocity magnitude
        if self.y_step > self.vmax:
            self.sp.velocity.y = self.vmax
        elif self.y_step < -self.vmax:
            self.sp.velocity.y = -self.vmax
        else:
            self.sp.velocity.y = self.y_step

        if self.local_pos.z < self.ALT_SP -0.5:
            self.sp.velocity.z = 0.25
        elif self.local_pos.z > self.ALT_SP +0.5:
            self.sp.velocity.z = -0.25
        else:
            self.sp.velocity.z = self.z_step
        
        # update yaw
        self.sp.yaw_rate = self.yaw_step
        
        #update previous velocities
        self.pre_sp_x = self.sp.velocity.x
        self.pre_sp_y = self.sp.velocity.y
        self.pre_sp_z = self.sp.velocity.z
        
        #print('jetson setpoint {:f} {:f} {:f} {:f}\n'.format(self.sp.velocity.x, self.sp.velocity.y, self.sp.velocity.z, self.sp.yaw_rate))
        self.bag.write('setpoint {:f} {:f} {:f} {:f} {:f} \n'.format(self.sp.velocity.x, self.sp.velocity.y, self.sp.velocity.z, self.sp.yaw_rate, rospy.Time.now().to_sec()))
   
    # define PID callback
    def execute_PID(self, msg):

        info_list = str(msg.data).split()

        info_list[0]=int(info_list[0])
        info_list[1]=int(info_list[1])
        info_list[2]=int(info_list[2])
        info_list[3]=float(info_list[3])
        
        #print('distance {:d} {:f}'.format(info_list[2], rospy.Time.now().to_sec()))
        self.bag.write('distance {:d} {:f}\n'.format(info_list[2], rospy.Time.now().to_sec()))
        #print('jetson object {:d} {:d}'.format(info_list[0], info_list[1]))
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
                #print('dt = ', self.dt)
                self.tic=self.toc
                
                self.x_step=0.1*self.x_pid_cnt.send((self.dt, info_list[2]-5000))
                self.y_step=self.y_pid_cnt.send((self.dt, center_offset[0]))
                self.z_step=self.z_pid_cnt.send((self.dt, center_offset[1]))
    
        else:
            self.no_detection+=1
            if self.no_detection>10:     
                self.x_step=0.0
                self.y_step=0.0
                self.z_step=0.0
            if self.no_detection>40:
                self.firstSight=True
            
        self.updateSp()
    	self.sp_pub.publish(self.sp)  
    	
    	  
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg
        self.bag.write('state '+self.state.mode+'\n')

    ## GPS callback
    def gpsCb(self, msg):
        self.gps = msg
        self.alt=self.gps.altitude
        #print('GPS {:f}, {:f}, {:f}'.format(self.gps.latitude, self.gps.longitude, self.alt))      
        #self.bag.write('GPS {:f} {:f} {:f}\n'.format(self.gps.latitude, self.gps.longitude, self.alt))
    
    def relaltCb(self, msg):
        self.rel_alt=msg.data
        #print("altitude {:f}".format(self.rel_alt))  
        #self.bag.write("altitude {:f}\n".format(self.rel_alt))         

# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()
    
    #reset pid controllers; zero the I
    global reset
    reset = True
    
    # controller object
    cnt = Controller()
    print('!!!! CONTROLLER INIT !!!!')
    
    reset=False
    
    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    # subscribe to gps data
    rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, cnt.gpsCb)
     # subscribe to gps data
    rospy.Subscriber('/mavros/global_position/rel_alt', Float64, cnt.relaltCb)
    
    #subscribe to yolo center offset and object's distance away
    rospy.Subscriber('detections', String, cnt.execute_PID)
    print("!!!! SUB / PUB INIT !!!!!")
    
    
    count=0
    #init_gps=[]
    #init_ra=[]
    init_pos=[]
    height=1.5
    time.sleep(10)
    while len(init_pos)<25:
        #init_gps.append(cnt.alt)
        init_pos.append(cnt.local_pos.z)
        #init_ra.append(cnt.rel_alt)
        time.sleep(0.2)
    
    #cnt.ALT_SP = sum(init_gps)/len(init_gps)+height
    #gps_mean = np.array(init_gps).mean()
    #ra_mean = np.array(init_ra).mean()
    pos_mean = np.array(init_pos).mean()
    #print('GPS average ', gps_data)
    cnt.ALT_SP = pos_mean + height
    print('!!!!!!!!!!!!!    INITIAL ALTITUDE ESTABLISHED    !!!!!!!!!!!')    
    while not rospy.is_shutdown():
        rospy.spin()
            
    print('shutting down yolo_pid')
    cnt.bag.close()
    print('yolo_pid bag closed')

    
if __name__ == '__main__':
    try:
        main()   
    except rospy.ROSInterruptException:
        pass  
