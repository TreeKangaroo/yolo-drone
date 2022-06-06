# An Autonomous Drone with Object Detection and Tracking Capabilities
---
![compressed_drone_gif](https://user-images.githubusercontent.com/72675667/172076750-efa2506a-fff8-4650-85dd-bdd58eacc917.gif)

This project develops an autonomous drone that can independently detect objects of interest and then fly closer to the object to take detailed photos. The drone hardware consists of a Holybro X500 drone development kit, a Nvidia Jetson Xavier NX GPU, and a RealSense camera. 3D printed fixtures were used to attach the GPU and camera to the drone frame. In software development, the Robot Operating System (ROS) was used, and three ROS nodes, ICODDA, pose estimation, and navigation control, were developed using Python. The ICODDA node performs image capture, object detection, and distance estimation. The YOLOv4 object detection network is integrated within the ICODDA node. A non-zero block average (NZBA) method was developed to estimate object distance in the presence of invalid and noisy depth data points. The pose estimation node implements a convolutional neural network that estimates the object’s pose based on depth data. It achieves an accuracy of 92% when validated with a dataset collected in the project. Finally, the navigation control node uses three PID (Proportional-Integral-Derivative) controllers to constantly adjust the drone’s speeds in the x, y, and z directions. It also implements a novel method to adjust the yaw speed based on recent pose estimation results. The drone’s autonomous operations were successfully demonstrated in test flights with different objects of interest. It achieves 24.4 frames per second image processing throughput and 0.034 second control latency. The developed technologies can be applied to autonomous drones in surveying, inspection, and search and rescue applications.

## Hardware Build
---
![drone](https://user-images.githubusercontent.com/72675667/172074330-7ff0d968-c56f-4e0d-9970-9335f7b6fd1d.jpg)

The drone is based on the Holybro X500 developer frame. It carries a Jetson Xavier NX (running Ubuntu 18.04) and an Intel Realsense d435i camera. The Jetson and camera fixtures are 3d-printed. The drone is powered by a 4 cell, 5200 mAH LiPo battery.

## Software Organization
---
- yolo_ros handles all YOLO related operations. You will need to download the pre-trained YOLO model online. Instructions can be found [here](https://github.com/indra4837/yolov4_trt_ros)
- navigation handles all drone movement and communication with the flight controller via MAVROS.
- pose contains the software needed to run a neural network that estimates bicycle pose based on depth data from within the bicycle's bounding box.

## Notes on software
---
**Important: Because I did not really know how ROS worked when I started this project, all the code files recognize the package name as yolov4_trt_ros. Due to the hassle of renaming ROS packages, I decided to just leave it. Before building a clone of this repo, it will probably be helpful to change the folder name to yolov4_trt_ros**
- There are quite a few codes in every file used for testing different components of the stack.
- There are also quite a few backup files from previous versions which may or may not work with how things are configured now.

## Software dependencies
---
- [ROS Melodic](http://wiki.ros.org/melodic) 
- [YOLO ROS implementation](https://github.com/indra4837/yolov4_trt_ros)
- [pyrealsense2](https://lieuzhenghong.com/how_to_install_librealsense_on_the_jetson_nx/) for Intel Realsense
- [MAVROS](https://docs.px4.io/v1.12/en/ros/mavros_installation.html)
- [cvbridge](http://wiki.ros.org/cv_bridge)
- Other standard python libraries (numpy, etc.)
- and all dependencies of these libraries

## References and Acknowledgements
---
- ROS implementation of YOLO with TensorRT acceleration from [indra4837](https://github.com/indra4837/yolov4_trt_ros)
- [MAVROS](http://wiki.ros.org/mavros)
- Intel Realsense python [library](https://github.com/IntelRealSense/librealsense)
- The wonderfully organized PX4 User Guide, particularly [development](https://docs.px4.io/v1.12/en/development/development.html), [robotics](https://docs.px4.io/v1.12/en/ros/ros1.html), and [build instructions](https://docs.px4.io/v1.12/en/frames_multicopter/holybro_x500_pixhawk4.html)
- PX4 forums and other online discussion
- [Garwin Family Foundation](https://garwinfamilyfoundation.org/) for project funding

