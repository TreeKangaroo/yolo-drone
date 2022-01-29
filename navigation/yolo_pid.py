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
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # setup step size for x, y, z direction
        self.x_step = 0.0
        self.y_step = 0.0
        self.z_step = 0.0
        
        self.yaw_step = 0.0
        
        # manuver label
        self.manuver = 'landed'
        
        self.gps = NavSatFix()
        self.alt=0.00
        self.rel_alt=0.00
        
        self.tic = rospy.Time.now().to_sec()
        self.toc = rospy.Time.now().to_sec()
        
        #new rosbag for data logging
        self.bag = open(r'/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/test.txt', 'w')
        
        
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        print('Current position ({:f}, {:f}, {:f})'.format(self.local_pos.x, self.local_pos.y, self.local_pos.z))
        self.bag.write('Current position ({:f}, {:f}, {:f})\n'.format(self.local_pos.x, self.local_pos.y, self.local_pos.z))
        self.toc = rospy.Time.now().to_sec()
        print('Current drone state: ', self.state.mode)
        self.tic = self.toc
    
    def updateSp(self):
        self.sp.velocity.x =self.x_step
        self.sp.velocity.y =self.y_step
        if self.local_pos.z < self.ALT_SP -0.5:
            self.sp.velocity.z = 0.25
        elif self.local_pos.z > self.ALT_SP +0.5:
            self.sp.velocity.z = -0.25
        else:
            self.sp.velocity.z = self.z_step
        
        # update yaw
        self.sp.yaw_rate = self.yaw_step
        
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg
        self.bag.write('Current drone state: '+self.state.mode+'\n')

    ## GPS callback
    def gpsCb(self, msg):
        self.gps = msg
        self.alt=self.gps.altitude
        print('GPS location ({:f}, {:f}, {:f})'.format(self.gps.latitude, self.gps.longitude, self.alt))      
        self.bag.write('GPS location ({:f}, {:f}, {:f})\n'.format(self.gps.latitude, self.gps.longitude, self.alt))
    
    def relaltCb(self, msg):
        self.rel_alt=msg.data
        print("Relative altitude: ({:f})".format(self.rel_alt))  
        self.bag.write("Relative altitude: ({:f})\n".format(self.rel_alt))  
        
    # functions
    def show_state(self):
        print('Current drone state: ', self.state)
        
    def show_positions(self):
        print('Current position ({:f}, {:f}, {:f})'.format(self.local_pos.x, self.local_pos.y, self.local_pos.z))           





# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()
    
    # controller object
    cnt = Controller()
    print('!!!! CONTROLLER INIT !!!!')

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

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    print("!!!! SUB / PUB INIT !!!!!")
    
    
    count=0
    init_gps=[]
    init_ra=[]
    init_pos=[]
    height=1.5
    time.sleep(10)
    while len(init_gps)<25:
        init_gps.append(cnt.alt)
        init_pos.append(cnt.local_pos.z)
        init_ra.append(cnt.rel_alt)
        time.sleep(0.2)
    
    #cnt.ALT_SP = sum(init_gps)/len(init_gps)+height
    gps_mean = np.array(init_gps).mean()
    ra_mean = np.array(init_ra).mean()
    pos_mean = np.array(init_pos).mean()
    #print('GPS average ', gps_data)
    cnt.ALT_SP = pos_mean + height
    
        
    #print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!         ALT SP = ', cnt.ALT_SP, '!!!!!!!!!!!!')
    cnt.bag.write('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!         average gps = ({:f})\n'.format(gps_mean))
    cnt.bag.write('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!         average relative altitude = ({:f})\n'.format(ra_mean))
    cnt.bag.write('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!         average local z position = ({:f})\n'.format(pos_mean))
        
    #move a bit hopefully not a lot
    while not rospy.is_shutdown():

        if cnt.state.mode == 'OFFBOARD':
            if count<40 or count>120:
                cnt.x_step=0.0
                cnt.y_step=0.0
                cnt.z_step=0.0
                cnt.yaw_step=0.0
            elif count>=40 and count<=120:
                cnt.x_step=0.0
                cnt.y_step=0.0
                cnt.z_step=0.0
                cnt.yaw_step=0.6
            count+=1
            
        else:
            cnt.x_step=0.0
            cnt.y_step=0.0
            cnt.z_step=0.0
            cnt.yaw_step=0.0
            count=0
            
        cnt.updateSp()
        cnt.bag.write('Setpoint velocity ({:f}, {:f}, {:f}, {:f})\n'.format(cnt.sp.velocity.x, cnt.sp.velocity.y, cnt.sp.velocity.z, cnt.sp.yaw_rate))
        sp_pub.publish(cnt.sp)
        rate.sleep()
            
            
    print('shutting down now')
    cnt.bag.close()
    print('bag closed')

    
if __name__ == '__main__':
    try:
        main()   
    except rospy.ROSInterruptException:
        pass  
