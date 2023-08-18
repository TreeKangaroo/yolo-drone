#!/usr/bin/env python3

import os
import time
import numpy as np
import queue

import cv2
import pycuda.autoinit  # For initializing CUDA driver
import pycuda.driver as cuda

from utils.yolo_classes import get_cls_dict
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
from utils.utility import get_distance
from src.tello_class import wingman
from djitellopy import Tello
import threading
print('!!!!!!! IMPORTED YOLO UTILS !!!!!')

import rospy
import rospkg
from msg import *

from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

color_img = [np.zeros((416, 416, 3), np.uint8), 0.0]
info_list = [0,0,0]
imgQ = queue.Queue(5)
infoQ = queue.Queue(5)

class yolov4(object):
    def __init__(self):
        """ Constructor """

        self.bridge = CvBridge()
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
            "tello_detections", Image, queue_size=1)

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


        #video=cv2.VideoWriter('/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/tello_video.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 15.0, (416,416))
                
        while not rospy.is_shutdown():
            try:

                if not imgQ.empty():
                    color_img =imgQ.get()
                    image_capture_time = color_img[1]
                    boxes, confs, clss = self.trt_yolo.detect(color_img[0], self.conf_th)
                    
                    self.toc = time.time()
                    self.times.append(self.toc - self.tic)
                    self.tic = self.toc
                                
                    self.times = self.times[-50:]
                    fps = len(self.times)/sum(self.times)
                    print('FPS_tello 1 = ', fps)
                    
                    self.publisher(color_img, boxes, confs, clss, image_capture_time)
                    
                    latency=self.toc - image_capture_time
                    print('latency_telllo 1= ', latency)
                    
                    if self.show_img:
                        cv_img = self.vis.draw_bboxes(color_img[0], boxes, confs, clss)
                        cv_img = show_fps(cv_img, fps)
                        #video.write(cv_img)
                        #cv2.imshow("YOLOv4 DETECTION RESULTS TELLO", cv_img)
                        #cv2.waitKey(1)
             
            except KeyboardInterrupt:
                rospy.on_shutdown(yolo.clean_up())
                
                print("Shutting Down")

        video.release()
        print('tello video close')
            # converts back to ros_img type for publishing
            #try:
           #     overlay_img = self.bridge.cv2_to_imgmsg(
                    #cv_img, encoding="passthrough")
               #rospy.logdebug("CV Image converted for publishing")
                #self.overlay_pub.publish(overlay_img)
           # except CvBridgeError as e:
               # rospy.loginfo("Failed to convert image %s", str(e))

    def publisher(self, color_img, boxes, confs, clss, image_capture_time):
        """ Publishes to detector_msgs

        Parameters:
        boxes (List(List(int))) : Bounding boxes of all objects
        confs (List(double))    : Probability scores of all objects
        clss  (List(int))    : Class ID of all classes
        """
        
        # find the bbox for the give class with the higest confidence level
        conf = 0
        bbox = [-1,-1,-1,-1]
        target_class = 1  # stop sign
        target_size = (120, 120)
        
        for i in range(len(boxes)):
            if clss[i]==target_class:
                if confs[i]>conf:
                    conf=confs[i]
                    bbox = boxes[i]
                    
        if bbox[0] != -1:
            x1, y1, x2, y2 = bbox
            dect_list=[int((x1+x2)/2), int((y1+y2)/2), 0, color_img[1]]

        else:
            dect_list=[0, 0, 0, color_img[1]]
        
        if not infoQ.full():
            infoQ.put(dect_list)

        

def main_yolo():

    yolo = yolov4()

    try:
        yolo.detect()
    except KeyboardInterrupt:
        rospy.on_shutdown(yolo.clean_up())
        print("Shutting down")

def main_wingman():
    
    #reset pid controllers; zero the I
    global reset
    reset = True
    
    # controller object
    wm = wingman(1)
    print('!!!! TELLO INIT !!!!')
    #rospy.Subscriber('tellocation', String, wm.location_cb)
    reset=False
    
    while True:
        try:

            color_img = wm.get_frame()
            if not imgQ.full():
                imgQ.put(color_img)

            if not infoQ.empty():
                wm.execute_PID(infoQ.get())

        except KeyboardInterrupt: 
            print('shutting down tello_pid')
            wm.bag.close()
            wm.tello.send_rc_control(0,0,0,0)

            wm.tello.end()
            break
    
if __name__ == '__main__':
    # initiate node
    rospy.init_node('wingman_node', anonymous=True)

    t1 = threading.Thread(target=main_yolo)
    t2 = threading.Thread(target=main_wingman)
    t1.start()
    t2.start()
