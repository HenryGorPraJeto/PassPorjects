#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool

count=0
sec=0
D=0
detected_sec=0
mom_detect=None
since=None


# this is the code that creating Distance estimation node and obtain tf infromation from ar_tag r
# ros package.  !!!! Run the launch file in AR_TAG rospackage called project.launch


def start_node():
    rospy.init_node('ar_tag')
    rospy.loginfo('image_subcriber node started')
    rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, Zcall_back,queue_size = 1)
    rospy.spin()
    

def Zcall_back(msg):
    global count
    global sec
    global D
    global detected_sec
    global mom_detect
    global since
    close=False


    data=msg.transforms[0]
    frame=data.child_frame_id
    if frame== 'camera':
        since=time.time()



    if frame == 'ar_marker_2':
        mom_detect=time.time()
        z=data.transform.translation.z
        D=z*100

    
    since_last=since-mom_detect


    if since_last > 3:
        D=99999999
    if D < 120 :
        close=True
    dis_pub=rospy.Publisher('Close',Bool,queue_size=1)
    dis_pub.publish(close)


    print('=========================')
    print('D is {}'.format(D))
    print(close)
   

 

    
    
    

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
