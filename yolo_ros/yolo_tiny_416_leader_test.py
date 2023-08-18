#!/usr/bin/env python3

import os
import time
import numpy as np

import cv2
import pycuda.autoinit  # For initializing CUDA driver
import pycuda.driver as cuda

from utils.yolo_classes import get_cls_dict
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
from utils.utility import get_distance
from src.realsense_source import camera_stream
print('!!!!!!! IMPORTED YOLO UTILS !!!!!')

import rospy
import rospkg
from msg import *
from std_msgs.msg import String




class yolov4(object):
    def __init__(self):
        """ Constructor """

        #self.bridge = CvBridge()
        self.init_params()
        print("!!!!!!!!!!!!!!!!!! PARAMS INIT !!!!!!!!!!!")
        self.init_yolo()
        print("!!!!!!!!!!!!!!!!!! YOLO INIT !!!!!!!!!!!")
        #self.cuda_ctx = cuda.Device(0).make_context()
        self.trt_yolo = TrtYOLO(
            (self.model_path + self.model), (self.h, self.w), self.category_num)
        self.tic = time.time()
        self.times = []
        print("Yolo Initialized")

    def __del__(self):
        """ Destructor """

        #self.cuda_ctx.pop()
        del self.trt_yolo
        #del self.cuda_ctx

    def clean_up(self):
        """ Backup destructor: Release cuda memory """

        if self.trt_yolo is not None:
            #self.cuda_ctx.pop()
            del self.trt_yolo
            #del self.cuda_ctx

    def init_params(self):
        """ Initializes ros parameters """
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("yolov4_trt_ros")
        self.video_topic = rospy.get_param("/video_topic", "/raw")
        self.model = rospy.get_param("/model", "yolov4-tiny-amalgamation")
        self.model_path = rospy.get_param(
            "/model_path", package_path + "/yolo_ros/yolo/")
        self.category_num = rospy.get_param("/category_number", 80)
        self.input_shape = rospy.get_param("/input_shape", "416")
        self.conf_th = rospy.get_param("/confidence_threshold", 0.5)
        self.show_img = rospy.get_param("/show_image", True)
        self.depth_only = rospy.get_param("/depth_only", True)
        self.detection_pub = rospy.Publisher(
            "detections", String, queue_size=1)
        self.tello_pub = rospy.Publisher("tellocation", String, queue_size=1)

    def init_yolo(self):
        """ Initialises yolo parameters required for trt engine """

        #if self.model.find('-') == -1:
        self.model = self.model + "-" + self.input_shape
            
        yolo_dim = self.model.split('-')[-1]

        if 'x' in yolo_dim:
            dim_split = yolo_dim.split('x')
            if len(dim_split) != 2:
                raise SystemExit('ERROR: bad yolo_dim (%s)!' % yolo_dim)
            self.w, self.h = int(dim_split[0]), int(dim_split[1])
        else:
            self.h = self.w = int(yolo_dim)
        if self.h % 32 != 0 or self.w % 32 != 0:
            raise SystemExit('ERROR: bad yolo_dim (%s)!' % yolo_dim)

        cls_dict = get_cls_dict(self.category_num)
        self.vis = BBoxVisualization(cls_dict)


    def detect(self):
        """Continuously capture images from camera and do object detection """

        # converts from ros_img to cv_img for processing
        camera = camera_stream()
        #video=cv2.VideoWriter('/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/video.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 15.0, (640,480))
                
        while not rospy.is_shutdown():
            try:
            
                color_img, depth_img = camera.get_frames()
                image_capture_time = rospy.Time.now().to_sec()
                
                boxes, confs, clss = self.trt_yolo.detect(color_img, self.conf_th)
                
                self.toc = time.time()
                self.times.append(self.toc - self.tic)
                self.tic = self.toc
                
                self.times = self.times[-20:]
                fps = len(self.times)/sum(self.times)
                print('FPS_leader = ', fps)

                self.publisher(color_img, depth_img, boxes, confs, clss, image_capture_time)
                
                latency=self.toc - image_capture_time
                print('latency= ', latency)
                
                if self.show_img:
                    cv_img = self.vis.draw_bboxes(color_img, boxes, confs, clss)
                    cv_img = show_fps(cv_img, fps)
                    #video.write(cv_img)
                    #cv2.imshow("YOLOv4 DETECTION RESULTS LEADER", cv_img)
                    #cv2.waitKey(1)
             
            except KeyboardInterrupt:
                rospy.on_shutdown(yolo.clean_up())
                print("Shutting Down")

        video.release()
        print('video close')
            # converts back to ros_img type for publishing
            #try:
           #     overlay_img = self.bridge.cv2_to_imgmsg(
                    #cv_img, encoding="passthrough")
               #rospy.logdebug("CV Image converted for publishing")
                #self.overlay_pub.publish(overlay_img)
           # except CvBridgeError as e:
               # rospy.loginfo("Failed to convert image %s", str(e))

    def publisher(self, color_img, depth_img, boxes, confs, clss, image_capture_time):
        """ Publishes to detector_msgs

        Parameters:
        boxes (List(List(int))) : Bounding boxes of all objects
        confs (List(double))    : Probability scores of all objects
        clss  (List(int))    : Class ID of all classes
        """
        
        # find the bbox for the given class with the higest confidence level
        conf_stop = 0
        conf_tello = 0
        bbox_stop = [-1,-1,-1,-1]
        bbox_tello = [-1,-1,-1,-1]
        #target_class = 1
        #target_size = (60, 60)
       
        for i in range(len(boxes)):
            if clss[i]==1:
                if confs[i]>conf_stop:
                    conf_stop=confs[i]
                    bbox_stop = boxes[i]
            elif clss[i]==0:
                if confs[i]>conf_tello:
                    conf_tello=confs[i]
                    bbox_tello = boxes[i]
        # If object detected, crop data within the bbox
        if bbox_stop[0] != -1:
            x1, y1, x2, y2 = bbox_stop
            depth_img = depth_img[y1:y2+1, x1:x2+1]
            distance = int(get_distance(depth_img, (9, 9), 10))
            #print(distance)
            cxs=int((x1+x2)/2)
            cys=int((y1+y2)/2)
            info_list = str(cxs)+ ' ' + str(cys) + ' ' + str(distance) + ' ' + str(image_capture_time)

        else:
            info_list='0 0 0 ' +str(image_capture_time)
        
        tello_loc = ''
        if bbox_tello[0] != -1:
            x1, y1, x2, y2 = bbox_tello
            #depth_img = depth_img[y1:y2+1, x1:x2+1]
            #distance = int(get_distance(depth_img, (9, 9), 10))
            #print(distance)
            cx=int((x1+x2)/2)
            cy=int((y1+y2)/2)
            #info_list = str(cx)+ ' ' + str(cy) + ' ' + str(distance) + ' ' + str(image_capture_time)
            
            #track tello location
            if cy<60:
                tello_loc = 'up '
            elif cy>356:
                tello_loc = 'down '
            
            if cx<60:
                tello_loc += 'left'
            elif cx>356:
                tello_loc += 'right'

        self.detection_pub.publish(info_list)
        if tello_loc != '':
            self.tello_pub.publish(tello_loc)
            #print('Location fense published')

def main():
    yolo = yolov4()
    rospy.init_node('yolov4_trt_ros', anonymous=True)
    try:
        yolo.detect()
    except KeyboardInterrupt:
        rospy.on_shutdown(yolo.clean_up())
        print("Shutting down")


if __name__ == '__main__':
    main()
