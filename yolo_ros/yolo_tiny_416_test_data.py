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
from src.tello_source import tello_stream
from tello_gt import *
print('!!!!!!! IMPORTED YOLO UTILS !!!!!')
import matplotlib.pyplot as plt
import rospy
import rospkg
from msg import *
#from msg import Detector2DArray
#from msg import Detector2D
from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


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
        self.model = rospy.get_param("/model", "yolov4-tiny-real")
        self.model_path = rospy.get_param(
            "/model_path", package_path + "/yolo_ros/yolo/")
        self.category_num = rospy.get_param("/category_number", 2)
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

        # converts from ros_img to cv_img for processing
        #camera = tello_stream()
       # video=cv2.VideoWriter('/home/michelle/catkin_ws/src/yolov4_trt_ros/bagfiles/video.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 15.0, (640,480))       
        #while not rospy.is_shutdown():
        tp=0
        tn=0
        fn=0
        fp=0
        num = 5
        #coordinates = np.zeros((num,3))
        dist_dif_list = []
        for i in range(0,7620):
            if i%1000==0:
                print(i)
            
            if vc05f_gt(i)==True:
                
                try:
                    img_name='vc05f'+str(i)
                    color_img = cv2.imread("/home/michelle/catkin_ws/src/yolov4_trt_ros/save_images/trial5/image/"+img_name+".jpg", cv2.IMREAD_COLOR)
                    depth_img = np.load("/home/michelle/catkin_ws/src/yolov4_trt_ros/save_images/trial5/depth/"+img_name+".npy")

                    yolo_result = False
                    boxes, confs, clss = self.trt_yolo.detect(color_img, self.conf_th)

                    """
                    if boxes.size !=0:
                        yolo_result=True
                        #print('YOLO: drone detected')
                    gt = vc05f_gt(i)
                    #print('Ground truth: ',gt)
                   # print('\n')
                    if yolo_result==True and gt==True:
                        tp+=1
                    if yolo_result==False and gt==False:
                        tn+=1
                    if yolo_result==True and gt==False:
                        fp+=1
                    if yolo_result==False and gt==True:
                        fn+=1    
                    #cv_img = self.vis.draw_bboxes(color_img, boxes, confs, clss)
                    #cv2.imshow("YOLOv4 DETECTION RESULTS TELLO", cv_img)
                    #cv2.waitKey(0)
                    #cv2.destroyWindow("YOLOv4 DETECTION RESULTS TELLO")
                    """
                    coordinates = self.publisher(color_img, depth_img, boxes, confs, clss)
                    if i!=437 and coordinates[0]!=0 and last_coordinates[0]!=0:
                        offset = coordinates - last_coordinates
                        dist_dif = (offset[0]**2 + offset[1]**2)**0.5
                        dist_dif_list.append(dist_dif/coordinates[2])
                    
                    last_coordinates = coordinates
                    #print(coordinates)

                except KeyboardInterrupt:
                    rospy.on_shutdown(yolo.clean_up())
                    print("Shutting Down")
        
        dist_dif_array = np.array(dist_dif_list)

        np.save('/home/michelle/catkin_ws/src/yolov4_trt_ros/yolo_ros/dist_dif_array.npy', dist_dif_array)
        print('saved')
        plt.hist(dist_dif_list, density=True, bins=30)
        plt.show()
        """        
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        xdata = coordinates[:,0].flatten()
        ydata = coordinates[:,1].flatten()
        zdata = coordinates[:,2].flatten()
        ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
        ax.plot3D(xdata, ydata, zdata)
        plt.show()
        
        print('True positive ',tp, 'rate ', tp/num)
        print('True negative ',tn, 'rate ', tn/num)
        print('False positive ', fp, 'rate ', fp/num)
        print('False negative ', fn, 'rate ', fn/num)
        """
        

        #video.release()
        #print('video close')
            # converts back to ros_img type for publishing
            #try:
           #     overlay_img = self.bridge.cv2_to_imgmsg(
                    #cv_img, encoding="passthrough")
               #rospy.logdebug("CV Image converted for publishing")
                #self.overlay_pub.publish(overlay_img)
           # except CvBridgeError as e:
               # rospy.loginfo("Failed to convert image %s", str(e))

    def publisher(self, color_img, depth_img, boxes, confs, clss):
        """ Publishes to detector_msgs

        Parameters:
        boxes (List(List(int))) : Bounding boxes of all objects
        confs (List(double))    : Probability scores of all objects
        clss  (List(int))    : Class ID of all classes
        """
        
        # find the bbox for the give class with the higest confidence level
        conf = 0
        bbox = [-1,-1,-1,-1]
        target_class = 0   # tello
        target_size = (120, 120)
        
        for i in range(len(boxes)):
            if clss[i]==target_class:
                if confs[i]>conf:
                    conf=confs[i]
                    bbox = boxes[i]
        """ 
        # If object detected, crop data within the bbox
        if bbox[0] != -1:
            x1, y1, x2, y2 = bbox
            depth_img = depth_img[y1:y2+1, x1:x2+1]
            distance = int(get_distance(depth_img, (9, 9), 10))
            
            depth_img = cv2.resize(depth_img, target_size)
            msa = (depth_img/256).astype(np.uint8)
            lsa = (depth_img - msa*256).astype(np.uint8)
            image_data = np.stack((msa, lsa), axis=2)
            if not self.depth_only:
                color_img = color_img[y1:y2+1, x1:x2+1]
                color_img = cv2.resize(color_img, target_size)
                image_data = np.concatenate((color_img, image_data), axis=2)
            
            image = self.bridge.cv2_to_imgmsg(image_data, encoding = 'passthrough')
            info_list=[int((x1+x2)/2), int((y1+y2)/2), distance]
        else:
            image = Image()
            info_list=[0,0,0]
        """
        if bbox[0] != -1:
            x1, y1, x2, y2 = bbox
            #print(x1,y1,x2,y2)
            #depth_img = depth_img[y1:y2+1, x1:x2+1]
            #print(depth_img)
            #distance = int(get_distance(depth_img, (2, 2), 0))
            distance =0
            info_list=[int((x1+x2)/2), int((y1+y2)/2), x2-x1]
            #image = Image()
        else:
            #image = Image()
            info_list=[0,0,0]
   
        #image.header.frame_id = " ".join(map(str,info_list))
        #image.header.stamp = image_capture_time
        #self.detection_pub.publish(image)
        return np.array(info_list)

def main():

    yolo = yolov4()
    rospy.init_node('yolov4_trt_ros', anonymous=True)
    print('ros node initialized')

    try:
        yolo.detect()
    except KeyboardInterrupt:
        rospy.on_shutdown(yolo.clean_up())
        print("Shutting down")


if __name__ == '__main__':
    main()
