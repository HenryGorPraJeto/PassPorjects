#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
import cv2
import math
import numpy as np
from custom_lanes_msg_python.msg import custom
from cv_bridge import CvBridge
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool
import time
bridge = CvBridge()



# This is the main control srcript that subcribing topic from all othe nodes such as
# YOLO , dash detection and ar_tag, control will be impletment base on thses information 
# from other nodes




count=0
sec=0
D=0.0
detected_sec=0
speed=0.049
dash_result='Not detected'
close=False

vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
curr_orientation_angle=0.0
phi=0.0

def start_node():
    rospy.init_node('lane_controller')
    rospy.loginfo('image_subcriber node started')
    rospy.Subscriber("/duckiebot/camera_node/image/compressed", CompressedImage, process_image)
    rospy.Subscriber('dash_result',String,dash_call_back)
    rospy.Subscriber("Close",Bool,close_call_back)
    rospy.Subscriber('Speed_control',Float64,speed_call_back)

    # rospy.Subscriber('y_and_Phi',Float64MultiArray,y_phi_call_back)
    rospy.spin()

def dash_call_back(msg):
    global dash_result
    global from_left
    global from_right
    dash_result=msg.data

def close_call_back(msg):
    global close
    close=msg.data
    

def speed_call_back(msg):
    global speed
    speed=msg.data
    print(speed)

def y_phi_call_back(msg):
    print('y and phi is ')
    recieved_data=msg.data
    global phi
    phi = recieved_data[1]
    print(phi)
    print(recieved_data)


def process_image(msg):
    global speed
    global close
    global D
    global dash_result
    
    try:
       #convert sensor_msgs/CompressedImage to OpenCV Image
        np_arr = np.fromstring(msg.data, np.uint8)
        orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        edge_filtered = detect_edges(orig)
        roi_filtered = region_of_interest(edge_filtered)
       #line detection 
        line_segments = detect_line_segments(roi_filtered)
        
        lane_lines = average_slope_intercept(orig, line_segments)
        #lane_lines = combine_lines (orig, line_segments)
        final_detected=lane_lines

        numbers_of_lanes=len(final_detected)
        pub3=rospy.Publisher('lane_coordination_custom_type',custom,queue_size=10)
        testmsg=custom()
        testmsg.testing_string.data='Detected_lanes'
        if numbers_of_lanes==2:
            list1=final_detected[0]
            list2=final_detected[1]
            list_one=list1[0]
            list_two=list2[0]
            final_list=[list_one,list_two]
            testmsg.list_one.data=list_one
            testmsg.list_two.data=list_two
            pub3.publish(testmsg)
        elif numbers_of_lanes==1:
            list1=final_detected[0]
            list_one=list1[0]
            final_list=[list_one]
        else:
            rospy.loginfo('no Detected_lanes')

       #draw detected lines on the origial image 
        lane_lines_image = display_lines(orig,lane_lines)
            
    except Exception as err:
        print err
    

    #publishing overlay image topic
    processed_img=bridge.cv2_to_imgmsg(lane_lines_image, "bgr8")
    pub=rospy.Publisher('lane_overlayed_image',Image,queue_size=10)
    pub.publish(processed_img)

    global curr_orientation_angle  
    robot_orientation_new=compute_heading_angle(orig, lane_lines)
    image_heading = display_heading_line( lane_lines_image,robot_orientation_new)
    curr_orientation_angle= curr_orientation_angle*0.0+ robot_orientation_new*1.0




    print(close)
    if close == False:
        # print('lane_follow')
        lane_controller( curr_orientation_angle,len(lane_lines)) 
        print('Distance is {}'.format(D))
    if close == True:
        print('over_take')
        overtake()
        print('Distance is {}'.format(D))

    show_image(image_heading)
    
def lane_controller(orientation,num_lane):
    global speed

    vel_reducer=(1-abs(orientation)/30.0)   #orig=30   
    if  vel_reducer < 0.0:
        vel_mul=0.0        
    else:
        vel_mul=vel_reducer #orig without *
        
    # proprotional controller values 
    kp_two_lines = -0.04 
    kp_single_line= -0.01  
    #  Due to the lagging issue the Duckiebot needed to be run and turn on a very slow speed
    # therfore the linear speed is hardcoded and the rotational speed is limited in certain range
    if num_lane==2:
        forward_vel=speed 
        angular_vel= kp_two_lines*float(orientation)
        if angular_vel > 0.65:
            angular_vel= 0.65
        if angular_vel < -0.65:
            angular_vel=-0.65

    elif num_lane==1:
        forward_vel=0.03
        angular_vel=kp_single_line*float(orientation)
        if angular_vel > 0.5:
            angular_vel=0.5
        if angular_vel <-0.5:
            angular_vel=-0.5
    else  :
        forward_vel=0.00
        angular_vel=0.00
    print(num_lane)
    print('X is {} truning is {}'.format(forward_vel,angular_vel))
    publishing_vel( forward_vel, angular_vel)




def overtake():
    global dash_result
    print('overtaking')
    angular_vel=0.0
    forward_vel=0.0

    #### overtake can not complete due to the lagging issue

    # if dash_result == 'left':
    #     print('overtaking from left')
    #     angular_vel=0.3
    #     time.sleep(2)
    #     forward_vel=0.05
    #     time.sleep(2)
        
        
    # if dash_result == 'right':
    #     print('overtaking from right')
    #     angular_vel=-0.3
    #     time.sleep(2)
    #     forward_vel=0.05
    #     time.sleep(2)
    
        
    print(angular_vel)
    publishing_vel(forward_vel,angular_vel)


def publishing_vel( forward_vel, angular_vel):
    vel = Twist()
    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = angular_vel
    vel.linear.x = forward_vel
    vel.linear.y = 0.0
    vel.linear.z = 0.0
    vel_pub.publish(vel)     
    

    
def detect_edges(frame):
    # filter for blue lane lines
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lower_lab=np.array([65, 135,100 ])
    upper_lab=np.array([160, 210, 250])
    lab_mask = cv2.inRange(lab, lower_lab, upper_lab)
    # detect edges
    # edges = cv2.Canny(mask, 200, 400)
    lab_edges=cv2.Canny(lab_mask, 200, 400)

    return lab_edges
    

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (width*1/6, 240),
        (width*5/6, 240),
        (width, height-20 ),
        (0, height-20  ),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges
    
    
def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 60  # angular precision in radian, i.e. 1 degree
    min_threshold = 20  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)                                    
    return line_segments
    
    
def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        rospy.loginfo('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                #rospy.loginfo('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            
            if slope < 0:               
               left_fit.append((slope, intercept))
            else:
               right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))
    

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    
    

    return lane_lines
    
    
    
def combine_lines (frame, line_segments):  
    color_1=[255, 0, 0] 
    color_2=[0, 0, 255] 
    thickness=2
    lane_lines = []
    if line_segments is None:
        rospy.loginfo('No line_segment segments detected')
        return lane_lines
    
    # state variables to keep track of most dominant segment
    largestLeftLineSize = 0
    largestRightLineSize = 0
    largestLeftLine = (0,0,0,0)
    largestRightLine = (0,0,0,0)
    left_lane_flag= False
    right_lane_flag= False
    
    for line in line_segments:
        for x1,y1,x2,y2 in line:
            size = math.hypot(x2 - x1, y2 - y1)
            slope = ((y2-y1)/(x2-x1))
            # Filter slope based on incline and
            # find the most dominent segment based on length
            if (slope > 0.0): #right
                if (size > largestRightLineSize):
                    largestRightLine = (x1, y1, x2, y2)
                    right_lane_flag= True                  
               #cv2.line(frame, (x1, y1), (x2, y2), color_1, thickness)
            elif (slope < 0.0): #left
                if (size > largestLeftLineSize):
                    largestLeftLine = (x1, y1, x2, y2)
                    left_lane_flag= True
               #cv2.line(frame, (x1, y1), (x2, y2), color_2, thickness)
                
    if  left_lane_flag: 
        x1,y1,x2,y2 = largestLeftLine         
        fit_left = np.polyfit((x1,y1), (x2,y2), 1) 
        cv2.line(frame, (x1, y1), (x2, y2), color_1, thickness)
        lane_lines.append(make_points(frame, fit_left))   
    
    if  right_lane_flag:
        x1,y1,x2,y2 = largestRightLine    
        fit_right = np.polyfit((x1,y1), (x2,y2), 1) 
        cv2.line(frame, (x1, y1), (x2, y2), color_2, thickness)  
        lane_lines.append(make_points(frame, fit_right))
    
    return lane_lines
    
def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]
    
    
def compute_heading_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = float(width / 2)
        x_offset = float( (left_x2 + right_x2) / 2 - mid)
        y_offset = float(height / 2)  
 
    elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = float(x2 - x1)
        y_offset = float(height / 2)

    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0.0
        y_offset = float(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = float(angle_to_mid_radian * 180.0 / math.pi)  
    heading_angle = angle_to_mid_deg 
    # rospy.loginfo("Heading angle %f :",angle_to_mid_deg)

    
    return heading_angle  
    
   
    
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image
    
    
def display_heading_line(frame, heading_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    
    heading_angle_radian = (heading_angle + 90.0) / 180.0 * math.pi 
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(heading_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

        
        
def show_image(img):
    cv2.imshow('ROI Filter', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
