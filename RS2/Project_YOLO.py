#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from std_msgs.msg import Float64

# This script is the create Node that obtain YOLO inforation from bounding box topic, run 
# launch file cal Project.launch in darknet ros folder
speed=0
def start_node():
    rospy.init_node('image_subcriber')
    rospy.loginfo('image_subcriber node started')
    rospy.Subscriber("/duckiebot/camera_node/image/compressed", CompressedImage, process_image,queue_size = 1)
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,boxes,queue_size=1)
    rospy.spin()


def boxes(msg):
    global speed

    for i in range(len(msg.bounding_boxes)):
        if msg.bounding_boxes[i].Class=='duckiebot':
            print('detectd ten')
            speed=0.04
        if msg.bounding_boxes[i].Class=='fifty':
            print('detectd fifty')
            speed=0.051
        if msg.bounding_boxes[i].Class=='ten':
            speed=0.05



    speed_pub=rospy.Publisher('Speed_control',Float64,queue_size=1)
    speed_pub.publish(speed)
    print(speed)


def process_image(msg):
    try:
       # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        orig=bridge.compressed_imgmsg_to_cv2(msg)

        # cv2.rectangle(orig,(xmin,ymin),(xmax,ymax),(200,255,100),2)
        # cv2.imshow('orig',orig)
        # cv2.waitKey(1)
           
    except Exception as err:
        print err
    # show results
    # show_image(roi_filtered)

 
def show_image(img):
    cv2.imshow('orig', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
