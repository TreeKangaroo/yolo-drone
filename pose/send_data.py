#!/usr/bin/env python3
# A realsense depth camera subsciber. 
# Subscribes to the /camera/color/image_raw and /camera/aligned_depth_to_color/image_raw topics

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
from rospy.numpy_msg import numpy_msg as np_msg
import pyrealsense2 as rs



def test():
	rospy.init_node('test_sending')
	bridge=CvBridge()
	pub=rospy.Publisher('/detections', Image, queue_size=1)
	
	width = 640
	height = 480
	rate = rospy.Rate(1)

	pipeline = rs.pipeline()
	config = rs.config()
	config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 15)
	config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 15)

	profile = pipeline.start(config)

	depth_sensor = profile.get_device().first_depth_sensor()
	depth_scale = depth_sensor.get_depth_scale()

	print("Depth Scale is: ", depth_scale)
	
	# create align object
	align_to = rs.stream.color
	align = rs.align(align_to)
    
	try:
		while not rospy.is_shutdown():
		    frames = pipeline.wait_for_frames()
		    aligned_frames = align.process(frames)
		    depth_frame = aligned_frames.get_depth_frame()
		    color_frame = aligned_frames.get_color_frame()
		    if not depth_frame or not color_frame:
		        continue

		    # convert images to numpy arrays
		    depth_array = np.asanyarray(depth_frame.get_data())
		    color_array = np.asanyarray(color_frame.get_data())
		    print(depth_array[123, 256])
		    
		    # split depth array to MSA and LSA
		    depth_array = depth_array[0:320, 0:320]          # crop a 320X320 block
		    msa = (depth_array/256).astype(np.uint8)
		    lsa = (depth_array - msa*256).astype(np.uint8)
		    depth_data = np.stack((msa, lsa), axis=2)
		    
		    # stack depth and color image together
		    #image_data = np.concatenate((color_array, depth_data), axis=2)
		    
		    
		    t1 = rospy.Time.now().to_sec()
		    color_image = bridge.cv2_to_imgmsg(depth_data, encoding='passthrough')
		    t2 = rospy.Time.now().to_sec()
		    print('cvBridge conversion time : ', t2-t1)
		    
		    color_image.header.stamp = rospy.Time.now()
		    bbox = (12, 200, 33, 168)
		    color_image.header.frame_id = " ".join(map(str,bbox))
		    
		    pub.publish(color_image)
		    t3 = rospy.Time.now().to_sec()
		    print('publish time : ', t3-t2)		 
		    rate.sleep()   

	finally:
		pipeline.stop()
		print('Pipeline is stopped')
	
if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
