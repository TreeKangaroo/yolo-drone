#!/usr/bin/env python
# ROS python API

import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Int16
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from utils.fcuModes import fcuModes


class Controller:
    def PID(self, Kp, Ki, Kd, MV_bar=0):
        e_prev = 0
        t_prev = -1
        I = 0
        MV = MV_bar
        while True:
            global reset
            if reset==True:
                I=0
            t, e = yield MV
            #e = SP - PV
            P = Kp * e
            I = I + Ki*e*(t - t_prev)
            D = Kd*(e - e_prev)/(t - t_prev)
            MV = MV_bar + P + I + D
            #print(f'MV:{int(MV)} P:{int(P)} I:{int(I)} D:{int(D)} e:{e} e_prev:{e_prev}')

            e_prev = e
            t_prev = t
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
        self.sp.velocity.x = 0.0
        self.sp.velocity.y = 0.0
        
        #initialize pid controllers
        self.x_pid_cnt = self.PID(0.6, 0.000001, 0.07)
        self.x_pid_cnt.send(None)
        
        self.y_pid_cnt = self.PID(0.6, 0.000001, 0.07)
        self.y_pid_cnt.send(None)
        
        self.z_pid_cnt = self.PID(0.6, -0.000001, 0.08, MV_bar=-0.1)
        self.z_pid_cnt.send(None)

        
        # initialize pid returned values
        self.x_pid_val = 0.0
        self.y_pid_val = 0.0
        self.z_pid_val = 0.0
        
        self.yaw_step = 0.0
        self.y_offset = 0.0
        
        # manuver label
        self.manuver = 'landed'
        
        # Setpoint publisher    
        self.sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
 

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.velocity.x =self.x_pid_val
        self.sp.velocity.y =self.y_pid_val
        if self.local_pos.z < 2.0:
            self.sp.velocity.z = 1.0
        else:
            self.sp.velocity.z = self.z_pid_val
        
        # update yaw
        self.sp.yaw_rate = self.yaw_step
        # displace current position
        #print('Current position ({:f}, {:f}, {:f})'.format(self.local_pos.x, self.local_pos.y, self.local_pos.z)) 

    # define PID callback
    def execute_PID(self, msg):
        print('receiving distance data ({:f}, {:f}, {:f})'.format(msg.x, msg.y, msg.z))
        t=rospy.Time.now().to_sec()
        #print('Lapped time: ', toc-tic, k, cnt.manuver)
        
        self.x_pid_val=self.x_pid_cnt.send((t, msg.x))
        self.y_pid_val=self.y_pid_cnt.send((t, msg.y))+self.y_offset
        self.z_pid_val=self.z_pid_cnt.send((t, msg.z))
        
        self.updateSp()
    	self.sp_pub.publish(self.sp)
   
   #Update Yaw step sizes and y velocity offsets discretely
   #NEEDS CHANGED
   #Positive yaw_rate means counterclockwise spin and vice versa
   def update_yaw(self, msg):
    if msg.data==1:
        self.yaw_step=-0.075
        self.y_offset = 0.05
    if msg.data==2:
        self.yaw_step=0.075
        self.y_offset = 0.05
    else:
        self.yaw_step=0
        self.y_offset=0.0

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

    reset=False

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # subscribe target distance NEEDS CHANGED
    rospy.Subscriber('target_distances', Point, cnt.execute_PID)
    rospy.Subscriber('poseLabel', Int16, cnt.update_yaw)
    
    

if __name__ == '__main__':
	try:
		main()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
