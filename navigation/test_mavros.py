#!/usr/bin/env python
# ROS python API

import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from utils.fcuModes import fcuModes

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('000111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # setup step size for x, y, z direction
        self.x_step = 0.0
        self.y_step = 0.0
        self.z_step = 0.0
        
        self.yaw_value = 0.0
        
        # manuver label
        self.manuver = 'landed'
        
        self.gps = GPSRAW()
        
        self.tic = rospy.Time.now().to_sec()
        self.toc = rospy.Time.now().to_sec()
        
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        print('receive position')
        self.toc = rospy.Time.now().to_sec()
        print('received state', 'update time: ', self.toc-self.tic)
        self.tic = self.toc
 

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg


    ## GPS callback
    def gpsCb(self, msg):
        self.gps = msg
        
    # functions
    def show_state(self):
        print('Current drone state: ', self.state)
        
    def show_positions(self):
        print('Current position ({:f}, {:f}, {:f})'.format(self.local_pos.x, self.local_pos.y, self.local_pos.z))      
    
    def show_gps(self):
        print('GPS location ({:f}, {:f}, {:f})'.format(self.gps.lat, self.gps.lon, self.gps.alt))      





# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    #rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    # subscribe to gps data
    #rospy.Subscriber('/mavros/global_position/raw/fix', GPSRAW, cnt.gpsCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


    # test if drone can be armed
    #while not cnt.state.armed:
        #modes.setArm()
        #rate.sleep()

    
    # show current state
    #cnt.show_state()
    
    # change mode
    #while not cnt.state.MODE_PX4_POSITION=='POSCTL':
    #    modes.setPositionMode()
    #    rate.sleep()
    
    # change mode
    #while cnt.state.armed:
     #   modes.setDisarm()
      #  rate.sleep()
        
    # show current state
    #cnt.show_state()
    
    
    # show current position
    cnt.show_positions()
    
    
    # show gps signal
    #cnt.show_gps()
    
    """
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    tic = rospy.Time.now().to_sec()
    k = 0
    while not rospy.is_shutdown():
        toc = rospy.Time.now().to_sec()
        print('Lapped time: ', toc-tic, k, cnt.manuver)
        tic = toc
        
        # make manuver
        if k<100:
            # move along x axis positive direction
            cnt.x_step = 0.5
            cnt.y_step = 0.0
            cnt.z_step = 0.0
            cnt.yaw_value = 0.0
            cnt.manuver = 'X forward'
            
        elif k<200:
            cnt.x_step = 0.0
            cnt.y_step = 0.0
            cnt.z_step = 0.0
            cnt.yaw_value = 2.0
            cnt.manuver = 'turn'
        
        elif k<300:
            cnt.x_step = 0.5
            cnt.y_step = 0.0
            cnt.z_step = 0.0
            cnt.yaw_value = 2.0
            cnt.manuver = 'X forward supposedly'
            
        else:
            k=0

        elif k<40:
            # move along y axis positive direction
            cnt.x_step = 0.0
            cnt.y_step = 0.0
            cnt.z_step = 0.0
            cnt.yaw_value = -2.0
            cnt.manuver = 'Y forward'   
        elif k<60:
            # move along x axis negative direction
            cnt.x_step = 0.0
            cnt.y_step = 0.0
            cnt.z_step = 0.0  
            cnt.yaw_value = 0
            cnt.manuver = 'X backward & assending'
        elif k<80:
            # move along y axis negative direction
            cnt.x_step = 0.0
            cnt.y_step = 0.0
            cnt.z_step = 0.0
            cnt.yaw_value = 3.0
            cnt.manuver = 'Y backward & desending'
        


        
        #cnt.yaw_value = 3
        k += 1     
    	cnt.updateSp()
    	sp_pub.publish(cnt.sp)
    	rate.sleep()
    	"""

if __name__ == '__main__':
	try:
		main()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
