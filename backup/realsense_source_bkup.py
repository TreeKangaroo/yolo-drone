#!/usr/bin/env python

# realsense RGB video feed for YOLO
# publishes on the topic ' '

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2

class camera_stream:
    
    def __init__(self):
        self.bridge = CvBridge()
        
        self.pub = rospy.Publisher('raw', Image, queue_size=2)
        self.msg = Image
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("Requires Depth camera with Color sensor")
            exit(0)
        
        self.config.enable_stream(rs.stream.color,640, 480, rs.format.bgr8, 15)
        self.pipeline.start(self.config)
        #print('started')
        
    def publish_frame_loop(self):
        while not rospy.is_shutdown():
            try:
                # Wait for color frame
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                # Convert image to numpy arrays and resize
                color_image = np.asanyarray(color_frame.get_data())
                #print('ugh')
                #color_image_resized = cv2.resize(color_image, (416, 416))
                #print('oof')

                #publish  
                self.pub.publish(self.bridge.cv2_to_imgmsg(color_image, encoding='passthrough'))
                #print('hola')

            except:
                continue
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('video_source') 
        cs = camera_stream()
        cs.publish_frame_loop()
    except rospy.ROSInterruptException:
        pass
    

