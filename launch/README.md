Launch multiple ros scripts at once using launch files. The format is roslaunch package_name launch_file. Example: roslaunch yolov4_trt_ros drone_launch_yolo.launch

drone_launch_yolo starts up yolo as well as all the drone navigation codes. The drone flies autonomously using this file
drone_launch only starts up drone systems. It does not start up yolo. The drone can fly on offboard mode, but not with object detection.
yolov4_trt starts everything associated with yolo. It does not have anything to do with the drone
