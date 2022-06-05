# An Autonomous Drone with Object Detection and Tracking Capabilities
---
This project develops an autonomous drone that can independently detect objects of interest and then fly closer to the object to take detailed photos. The drone hardware consists of a Holybro X500 drone development kit, a Nvidia Jetson Xavier NX GPU, and a RealSense camera. 3D printed fixtures were used to attach the GPU and camera to the drone frame. In software development, the Robot Operating System (ROS) was used, and three ROS nodes, ICODDA, pose estimation, and navigation control, were developed using Python. The ICODDA node performs image capture, object detection, and distance estimation. The YOLOv4 object detection network is integrated within the ICODDA node. A non-zero block average (NZBA) method was developed to estimate object distance in the presence of invalid and noisy depth data points. The pose estimation node implements a convolutional neural network that estimates the object’s pose based on depth data. It achieves an accuracy of 92% when validated with a dataset collected in the project. Finally, the navigation control node uses three PID (Proportional-Integral-Derivative) controllers to constantly adjust the drone’s speeds in the x, y, and z directions. It also implements a novel method to adjust the yaw speed based on recent pose estimation results. The drone’s autonomous operations were successfully demonstrated in test flights with different objects of interest. It achieves 24.4 frames per second image processing throughput and 0.034 second control latency. The developed technologies can be applied to autonomous drones in surveying, inspection, and search and rescue applications.

## References and Acknowledgements
---
- ROS implementation of YOLO with TensorRT acceleration from [indra4837](https://github.com/indra4837/yolov4_trt_ros)
- [MAVROS](http://wiki.ros.org/mavros)
- Intel Realsense python [library](https://github.com/IntelRealSense/librealsense)
- The wonderfully organized PX4 User Guide, particularly [development](https://docs.px4.io/v1.12/en/development/development.html), [robotics](https://docs.px4.io/v1.12/en/ros/ros1.html), and [build instructions](https://docs.px4.io/v1.12/en/frames_multicopter/holybro_x500_pixhawk4.html)
- PX4 forums and other online discussion
- [Garwin Family Foundation](https://garwinfamilyfoundation.org/) for project funding

