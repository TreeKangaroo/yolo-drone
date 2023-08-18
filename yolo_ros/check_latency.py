#!/usr/bin/env python
# ROS python API

import rospy
from sensor_msgs.msg import Image
import time

class listener:

    def __init__(self):
        self.lat_list = []  

    def img_cb(self, msg):
        time=msg.header.stamp.to_sec()
        cur_time=rospy.Time.now().to_sec()
        self.lat_list.append(cur_time-time)
        self.lat_list=self.lat_list[-50:]
        
        print('latency= ', sum(self.lat_list)/len(self.lat_list))

  
rospy.init_node('latency') 
l=listener()
rospy.Subscriber('detections', Image, l.img_cb)
rospy.spin()





