#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String



def start_node():
    rospy.init_node('dash_detect')
    rospy.loginfo('image_subcriber node started')
    rospy.Subscriber("/duckiebot/camera_node/image/compressed", CompressedImage, process_image,queue_size = 1)
    rospy.spin()
    
def process_image(msg):
    try:
       # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        orig=bridge.compressed_imgmsg_to_cv2(msg)
        color_filtered=detect_contours(orig)
        roi_filtered = region_of_interest(color_filtered)

        # four points
        #top left
        point1=(150,240)
        cv2.circle(orig,point1,5,(255,0,0),-1)
        #top right
        point2=(490,240)
        cv2.circle(orig,point2,5,(255,0,0),-1)
        #bottom left
        point3=(0,400)
        cv2.circle(orig,point3,5,(255,0,0),-1)
        #bottom right
        point4=(640,400)
        cv2.circle(orig,point4,5,(255,0,0),-1)
 # 
        pts1=np.float32([[150,240],[490,240],[0,400],[640,400]])
        pts2=np.float32([[0,0],[400,0],[0,600],[400,600]])
        matrix=cv2.getPerspectiveTransform(pts1,pts2)
        result=cv2.warpPerspective(roi_filtered,matrix,(400,600))
        IPM_orig=cv2.warpPerspective(orig,matrix,(400,600))


        _,contours,_=cv2.findContours(result,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        print('the total number of contours is {} '.format(len(contours)))
        left_turn_OK=False
        right_turn_OK=False
        allow_change_lane=False
        big_contours=[]
        small_contours=[]
        big_contours_centers=[]
        small_contours_centers=[]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000 and area<10000:
                x,y,w,h=cv2.boundingRect(contour)
                cv2.rectangle(IPM_orig,(x,y),(x+w,y+h),(0,0,255),2)
                center_x=x+(w/2)
                center_y=y+(h/2)
                center=[center_x,center_y]
                cv2.circle(IPM_orig,(center_x,center_y),5,(255,0,0),-1)
                small_contours.append(contour)
                small_contours_centers.append(center)

            if area > 12000:
                x1,y1,w1,h1 =cv2.boundingRect(contour)
                cv2.rectangle(IPM_orig,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
                center_x1=x1+(w1/2)
                center_y1=y1+(h1/2)
                cv2.circle(IPM_orig,(center_x1,center_y1),5,(255,0,255),-1)
                center1=[center_x1,center_y1]
                big_contours.append(contour)
                big_contours_centers.append(center1)

                
        
        
        cv2.drawContours(IPM_orig,small_contours,-1,(0,0,255),3)
        cv2.drawContours(IPM_orig,big_contours,-1,(0,255,0),3)

        truning_direction='NONE'
        if len(small_contours) > 0:
            small_average=np.average(small_contours_centers,axis=0)
            print(small_average)
            if small_average[0] < 200 :
                truning_direction= 'left'
            if small_average[0] > 200 :
                truning_direction= 'right'
        print(truning_direction)
        dash_pub=rospy.Publisher('dash_result',String,queue_size=10)
        dash_pub.publish(truning_direction)




        # print('num_small is {} num_big is {}'.format(len(small_contours_centers),len(big_contours_centers)))
        # cv2.imshow('contours',orig)
        # cv2.imshow('IPM',result)
        # cv2.imshow('IPM_ORIG',IPM_orig)
        # cv2.waitKey(1)
        
    except Exception as err:
        print err
    # show results
    # show_image(roi_filtered)
    

def detect_contours(frames):

    blurred_frame=cv2.GaussianBlur(frames,(5,5),0)
    hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
    lower_blue = np.array([65, 135, 100])
    upper_blue = np.array([150, 210, 230])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)


    return mask
 





    

def region_of_interest(edges):


    height, width = edges.shape
    #print('height {} and width {}' .format(height,width))

    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    # [top-left,top-right,bottom-left,bottom-right]

    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height ),
        (0, height ),
    ]], np.int32)

   

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges
        
        
def show_image(img):
    cv2.imshow('ROI Filter', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
