#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def callback(msg):
    print(msg.header.frame_id)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('detections', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
