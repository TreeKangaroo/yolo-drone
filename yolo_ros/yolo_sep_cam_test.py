#!/usr/bin/env python3

import os
import time

import cv2
import pycuda.autoinit  # For initializing CUDA driver
import pycuda.driver as cuda

from utils.yolo_classes import get_cls_dict
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO

import rospy
import rospkg
from yolov4_trt_ros.msg import Detector2DArray
from yolov4_trt_ros.msg import Detector2D
from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

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
        self.times = []
        print('Yolo Initialized')

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
        self.video_topic = rospy.get_param("/video_topic", "raw")
        self.model = rospy.get_param("/model", "yolov4")
        self.model_path = rospy.get_param(
            "/model_path", package_path + "/yolo_ros/yolo/")
        self.category_num = rospy.get_param("/category_number", 80)
        self.input_shape = rospy.get_param("/input_shape", "288")
        self.conf_th = rospy.get_param("/confidence_threshold", 0.5)
        self.show_img = rospy.get_param("/show_image", False)
        #self.image_sub = rospy.Subscriber(
            #self.video_topic, Image, self.img_callback, queue_size=1, buff_size=1920*1080*3)
        self.image_sub = rospy.Subscriber(
            self.video_topic, Image, self.img_callback, queue_size=1, buff_size=640*480*10)
        self.detection_pub = rospy.Publisher(
            "detections", Image, queue_size=1)
        self.overlay_pub = rospy.Publisher(
            "/result/overlay", Image, queue_size=1)

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


    def img_callback(self, ros_img):
        """Continuously capture images from camera and do object detection """

        # converts from ros_img to cv_img for processing
        try:
            #print('enter detect')
            img_capture_time=ros_img.header.stamp
            cv_img = self.bridge.imgmsg_to_cv2(
                ros_img, desired_encoding="passthrough")
            
            color_img = cv_img[:,:, 0:3]
            depth_img = cv_img[:,:, 3:5]
            
            #print("ROS Image converted for processing")
        except CvBridgeError as e:
            print("Failed to convert image %s", str(e))

        if color_img is not None:

            boxes, confs, clss = self.trt_yolo.detect(color_img, self.conf_th)
            #print('detected')
            cv_img = self.vis.draw_bboxes(color_img, boxes, confs, clss)
            self.publisher(color_img, depth_img, boxes, confs, clss, img_capture_time)
           
            
            self.toc = time.time()
            self.times.append(self.toc - self.tic)
            self.tic = self.toc
                        
            self.times = self.times[-50:]
            fps = len(self.times)/sum(self.times)
            print('FPS = ', fps)

            

            if self.show_img:
                cv_img = show_fps(color_img, fps)
                cv2.imshow("YOLOv4 DETECTION RESULTS", cv_img)
                cv2.waitKey(1)
        """
        # converts back to ros_img type for publishing
        try:
            overlay_img = self.bridge.cv2_to_imgmsg(
                cv_img, encoding="passthrough")
            rospy.logdebug("CV Image converted for publishing")
            self.overlay_pub.publish(overlay_img)
        except CvBridgeError as e:
            rospy.loginfo("Failed to convert image %s", str(e))
        """
        
    def publisher(self, color_img, depth_img, boxes, confs, clss, image_capture_time):
            """ Publishes to detector_msgs

            Parameters:
            boxes (List(List(int))) : Bounding boxes of all objects
            confs (List(double))    : Probability scores of all objects
            clss  (List(int))    : Class ID of all classes
            """
            
            # find the bbox for the give class with the higest confidence level
            conf = 0
            bbox = [-1,-1,-1,-1]
            target_class = 11
            target_size = (60, 60)
            
            for i in range(len(boxes)):
                if clss[i]==target_class:
                    if confs[i]>conf:
                        conf=confs[i]
                        bbox = boxes[i]
            
            # If object detected, crop data within the bbox
            if bbox[0] != -1:
                x1, y1, x2, y2 = bbox
                depth_img = depth_img[y1:y2+1, x1:x2+1]
                #distance = int(get_distance(depth_img, (9, 9), 10))
                
                print('detect')
                depth_img = cv2.resize(depth_img, target_size)
                #if not self.depth_only:
                #    color_img = color_img[y1:y2+1, x1:x2+1]
                #    color_img = cv2.resize(color_img, target_size)
                #    image_data = np.concatenate((color_img, image_data), axis=2)
                
                image = self.bridge.cv2_to_imgmsg(depth_img, encoding = 'passthrough')
                #info_list=[int((x1+x2)/2), int((y1+y2)/2), distance]
            else:
                image = Image()
                #info_list=[0,0,0]
            
            #image.header.frame_id = " ".join(map(str,info_list))
            image.header.stamp = image_capture_time
            self.detection_pub.publish(image)


def main():
    rospy.init_node('yolov4_trt_ros', anonymous=True)
    yolo = yolov4()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.on_shutdown(yolo.clean_up())
        print("Shutting down")


if __name__ == '__main__':
    main()
