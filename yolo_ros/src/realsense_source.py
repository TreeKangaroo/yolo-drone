#!/usr/bin/env python3

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
        #self.bridge = CvBridge()
        
        #self.pub = rospy.Publisher('raw', Image, queue_size=1)
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        
        self.align = rs.align(rs.stream.color)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) 
        self.config.enable_stream(rs.stream.color,640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        #print('started')
        
    def get_frames(self):
        success=False
        while not success:
            try:
                # Wait for color frame
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if depth_frame and color_frame:
                    success=True
                    
            except:
                continue
         # Convert image to numpy arrays and resize
        color_array = np.asanyarray(color_frame.get_data())
        depth_array = np.asanyarray(depth_frame.get_data())
       
        return color_array, depth_array
        
if __name__ == '__main__':
    try:
        rospy.init_node('video_source') 
        cs = camera_stream()
        cs.publish_frame_loop()
    except rospy.ROSInterruptException:
        pass
    

