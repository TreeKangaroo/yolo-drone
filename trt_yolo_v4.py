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
from src.realsense_source import camera_stream

import rospy
import rospkg
from yolov4_trt_ros.msg import Detector2DArray
from yolov4_trt_ros.msg import Detector2D
from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class yolov4(object):
    def __init__(self):
        """ Constructor """

        self.bridge = CvBridge()
        self.init_params()
        self.init_yolo()
        self.cuda_ctx = cuda.Device(0).make_context()
        self.trt_yolo = TrtYOLO(
            (self.model_path + self.model), (self.h, self.w), self.category_num)
        self.tic = time.time()
        print("Yolo Initialized")

    def __del__(self):
        """ Destructor """

        self.cuda_ctx.pop()
        del self.trt_yolo
        del self.cuda_ctx

    def clean_up(self):
        """ Backup destructor: Release cuda memory """

        if self.trt_yolo is not None:
            self.cuda_ctx.pop()
            del self.trt_yolo
            del self.cuda_ctx

    def init_params(self):
        """ Initializes ros parameters """
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("yolov4_trt_ros")
        self.video_topic = rospy.get_param("/video_topic", "/raw")
        self.model = rospy.get_param("/model", "yolov4Custom")
        self.model_path = rospy.get_param(
            "/model_path", package_path + "/yolo/")
        self.category_num = rospy.get_param("/category_number", 80)
        self.input_shape = rospy.get_param("/input_shape", "288")
        self.conf_th = rospy.get_param("/confidence_threshold", 0.5)
        self.show_img = rospy.get_param("/show_image", False)
        #self.image_sub = rospy.Subscriber(
            #self.video_topic, Image, self.img_callback, queue_size=1, buff_size=1920*1080*3)
        #self.image_sub = rospy.Subscriber(
            #self.video_topic, Image, self.img_callback, queue_size=1, buff_size=640*480*4)
        self.detection_pub = rospy.Publisher(
            "detections", Image, queue_size=1)
        #self.yolo_pub = rospy.Publisher(
            #"/result/overlay", Image, queue_size=1)

    def init_yolo(self):
        """ Initialises yolo parameters required for trt engine """

        if self.model.find('-') == -1:
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
        
        while not rospy.is_shutdown():
            try:
            
                color_img, depth_img = camera.get_frames()
                t1=rospy.Time.now()
                boxes, confs, clss = self.trt_yolo.detect(color_img, self.conf_th)
                
                self.toc = time.time()
                fps = 1.0 / (self.toc - self.tic)
                print("FPS: ", fps)
                self.tic = self.toc

                self.publisher(color_img, depth_img, boxes, confs, clss, t1)
                
                if self.show_img:
                    cv_img = self.vis.draw_bboxes(color_img, boxes, confs, clss)
                    cv_img = show_fps(cv_img, fps)
                    cv2.imshow("YOLOv4 DETECTION RESULTS", cv_img)
                    cv2.waitKey(1)
             
            except KeyboardInterrupt:
                rospy.on_shutdown(yolo.clean_up())
                print("Shutting Down")


            # converts back to ros_img type for publishing
            #try:
           #     overlay_img = self.bridge.cv2_to_imgmsg(
                    #cv_img, encoding="passthrough")
               #rospy.logdebug("CV Image converted for publishing")
                #self.overlay_pub.publish(overlay_img)
           # except CvBridgeError as e:
               # rospy.loginfo("Failed to convert image %s", str(e))

    def publisher(self, color_img, depth_img, boxes, confs, clss, t1):
        """ Publishes to detector_msgs

        Parameters:
        boxes (List(List(int))) : Bounding boxes of all objects
        confs (List(double))    : Probability scores of all objects
        clss  (List(int))    : Class ID of all classes
        """
        #stack color and depth images
        msa = (depth_img/256).astype(np.uint8)
        lsa = (depth_img - msa*256).astype(np.uint8)
        depth_data = np.stack((msa, lsa), axis=2)
        
        image_data = np.concatenate((color_img, depth_data), axis=2)
        
        conf = 0
        bbox = [-1,-1,-1,-1]
        
        for i in range(len(boxes)):
            if clss[i]==56:
                if confs[i]>conf:
                    conf=confs[i]
                    bbox = boxes[i]
        
        """   
        detection2d = Detector2DArray()
        detection = Detector2D()
        detection2d.header.stamp = rospy.Time.now()
        detection2d.header.frame_id = "camera" #change accordingly
        
        for i in range(len(boxes)):
            # boxes : xmin, ymin, xmax, ymax
            for _ in boxes:
                detection.header.stamp = rospy.Time.now()
                detection.header.frame_id = "camera" # change accordingly
                detection.results.id = clss[i]
                detection.results.score = confs[i]
                
                detection.bbox.center.x = boxes[i][0] + (boxes[i][2] - boxes[i][0])/2
                detection.bbox.center.y = boxes[i][1] + (boxes[i][3] - boxes[i][1])/2
                detection.bbox.center.theta = 0.0  # change if required
                
                detection.bbox.size_x = abs(boxes[i][0] - boxes[i][2])
                detection.bbox.size_y = abs(boxes[i][1] - boxes[i][3])

            detection2d.detections.append(detection)
        
        self.detection_pub.publish(detection2d)
        """
        #image_data = image_data[bbox[1]:bbox[3]+1, bbox[0]:bbox[2], :]
        image = self.bridge.cv2_to_imgmsg(image_data, encoding = 'passthrough')
        image.header.frame_id = " ".join(map(str,bbox))
        image.header.stamp = t1
        self.detection_pub.publish(image)
        

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
