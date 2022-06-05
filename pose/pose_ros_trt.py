#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int16
import cv2
import numpy as np

import tensorrt as trt 
import pycuda.driver as cuda
import pycuda.autoinit


class TrtPose:

    def load_engine(self, plan_path):
        with open(plan_path, 'rb') as f:
            engine_data = f.read()
        self.engine = self.trt_runtime.deserialize_cuda_engine(engine_data) 
    
    def allocate_buffers(self, data_type):
        # Determine dimensions and create page-locked memory buffers (which won't be swapped to disk) to hold host inputs/outputs.
        self.h_input = cuda.pagelocked_empty(trt.volume(self.engine.get_binding_shape(0)), dtype=trt.nptype(data_type))
        self.h_output = cuda.pagelocked_empty(trt.volume(self.engine.get_binding_shape(1)), dtype=trt.nptype(data_type))
        # Allocate device memory for inputs and outputs.
        self.d_input = cuda.mem_alloc(self.h_input.nbytes)
        self.d_output = cuda.mem_alloc(self.h_output.nbytes)
        # Create a stream in which to copy inputs/outputs and run inference.
        self.stream = cuda.Stream()
    
    def __init__(self):
        self.cuda_ctx = cuda.Device(0).make_context()
        self.TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        self.trt_runtime = trt.Runtime(self.TRT_LOGGER)
        self.load_engine('/home/michelle/catkin_ws/src/yolov4_trt_ros/pose/model_ds_60.plan')
        self.context = self.engine.create_execution_context()
        self.allocate_buffers(trt.float32)
        
    def __del__(self):
        """Free CUDA memories."""
        self.cuda_ctx.pop()
        #self.context.pop()
        del self.h_input
        del self.h_output
        del self.d_input
        del self.d_output
        del self.stream
        del self.cuda_ctx
        del self.context

    def detect(self, data):
        preprocessed = np.asarray(data).ravel()
        np.copyto(self.h_input, preprocessed) 

        if self.cuda_ctx:
            self.cuda_ctx.push()
            
        cuda.memcpy_htod_async(self.d_input, self.h_input, self.stream)

        # Run inference.
        self.context.profiler = trt.Profiler()
        self.context.execute(batch_size=1, bindings=[int(self.d_input), int(self.d_output)])

        # Transfer predictions back from the GPU.
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
        
        # Synchronize the stream
        self.stream.synchronize()
        
        # Return the host output.
        out = self.h_output
                       
        if self.cuda_ctx:
            self.cuda_ctx.pop()
            
        return out

            
class pose:
    # constructor
    def __init__(self):
        self.bridge = CvBridge()
        #self.cuda_ctx = cuda.Device(0).make_context()
        self.trt_pose = TrtPose()
        
        self.img_sub = rospy.Subscriber('/detections', Image, self.callback_detection, queue_size=1, buff_size=60*60*3)
        #self.pose_pub = rospy.Publisher('/poseLabel', Int16, queue_size=1)
        
        self.bag = open(r'/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/pose.txt', 'w')

    def __del__(self):
        """ Destructor """

        #self.cuda_ctx.pop()
        del self.trt_pose
        #del self.cuda_ctx

    def clean_up(self):
        """ Backup destructor: Release cuda memory """

        if self.trt_pose is not None:
            #self.cuda_ctx.pop()
            del self.trt_pose
            #del self.cuda_ctx    

    def callback_detection(self, ros_data):  
           
        img = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding='passthrough')

        depth_data = img[:, :, 0]*256 + img[:, :, 1]
    
        show_image = False
        if show_image:
            image = cv2.applyColorMap(cv2.convertScaleAbs(depth_data, alpha=0.03), cv2.COLORMAP_JET)
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', image)        
            cv2.waitKey(1)

        # perform pose detection
        depth_data = (depth_data/8000).astype(np.float32)
        depth_data[depth_data>1.0]=1.0
        depth_data = np.expand_dims(depth_data, axis=(0, 3))
        out = self.trt_pose.detect(depth_data)
        self.bag.write('pose {:d} {:f}\n'.format(np.argmax(out), rospy.Time.now().to_sec()))
        print(np.argmax(out))
       

def main():
    pose_detect = pose()    
    rospy.init_node('Pose_Detection', anonymous=True)
    
    while not rospy.is_shutdown():
        rospy.spin()
            
    print('shutting down pose estimation')
    pose_detect.bag.close()
    print('pose bag closed') 
      

if __name__ == '__main__':
    main()
