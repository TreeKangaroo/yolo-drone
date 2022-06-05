IMPORTANT: run source ~/cvbridge_build_ws/install/setup.bash in terminal first
This command must be ran or else cvbridge will not work. However, this command does not play nice with any of the yolo code, so pose estimation network must be ran in its OWN TERMINAL.This is why pose estimation is not included in any launch file.

detect.py:          Perform inference with donModel.plan with multiple files
detect2.py:         create pose class and perform inference with a single file
detect3.py:         perform inference with onnx model

pose_ros_onnx.py:   ros node for pose detection using onnx model. 
pose_ros_trt.py:    ros node for pose detection using trt model. It is faster than onnx model. This one actually works during a drone flight and publishes the pose.

model_ds_60.plan:  This is a legitimate trained model for bicycle pose estimation

