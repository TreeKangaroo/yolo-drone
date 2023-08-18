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

import engine as eng
import inference as inf
import tensorrt as trt 


class pose(object):
    # constructor
    def __init__(self):
        self.bridge = CvBridge()
        self.TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        self.trt_runtime = trt.Runtime(self.TRT_LOGGER)
        self.engine = eng.load_engine(self.trt_runtime, 'donModel.plan')
        self.h_input, self.d_input, self.h_output, self.d_output, self.stream = inf.allocate_buffers(self.engine, trt.float32)
        
        self.img_sub = rospy.Subscriber('/detections', Image, self.callback_detection, queue_size=1, buff_size=640*480*6)
        
    
    def callback_detection(self, ros_data):  
        # get bonding box
        bbox = [int(x) for x in ros_data.header.frame_id.split()]
    
        if bbox[0] != -1:      # object of interest detected
            img = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding='passthrough')
            # reconstuct depth data and separate color image and depth data
            r, c, z = img.shape
            if z == 5:
                color_image = img[:, :, :3]
                depth_data = img[:, :, 3]*256 + img[:, :, 4]
            else:
                depth_data = img[:, :, 0]*256 + img[:, :, 1]
        
            show_image = True
            if show_image:
                if z ==5:
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_data, alpha=0.03), cv2.COLORMAP_JET)
                    image = np.hstack((color_image, depth_colormap))
                else:
                    image = cv2.applyColorMap(cv2.convertScaleAbs(depth_data, alpha=0.03), cv2.COLORMAP_JET)
            
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', image)        
                cv2.waitKey(1)

            # perform pose detection
            depth_data = (depth_data/65536).astype(np.float32)
            depth_data = np.expand_dims(depth_data, axis=(0, 3))
            out = inf.do_inference(self.engine, depth_data, self.h_input, self.d_input, self.h_output, self.d_output, self.stream)
            print(out)
        else:
            print('No object detected')
        

def main():
    pose_detect = pose()
    rospy.init_node('Pose_Detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        #rospy.on_shutdown(yolo.clean_up())
        cv2.destroyAllWindows()
        print("Shutting down")    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
