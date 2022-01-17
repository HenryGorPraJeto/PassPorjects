This folder contain condes done by student for the Final Project.


The LAB_duckie_dash_detection.py file is the script that create the dash detection node and subcribering the /duckiebot/camera_node/image/compress image from duckiebotand use IPM technique to do dash line detection 


The Project_YOLO.py is the script that create the sign detection and speed control node which this node will subcribe the yolo bounding box topic and publish the speed control information (darknet_ros package and model weight files not provided).


the ar_tag.py file is the script that create the distance estimation node which the node is subcribe to !!!!!TF!!!!!(not marker pose) topic (launch the ar_tag rospackage via project_ar.launch file provide) and publish a Boolean value.  in this project, student use ar_tag id 2. if want to reuse on other project and different ar_tage. please modify ar_tag name


the lane_follow.py file script that create the lane follow node which subcribe all topics that stated above and use those information to make decision what the duckiebot should do and publish linear and angular velocity to topic /cmd_vel topic 


