#include "sample.h"
#include <chrono>
#include "std_msgs/Bool.h"
#include <cmath>
using namespace std;

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Subscribing to images (which require a image transport interpreter)
 * - Publishing images
 * - The need to exchange data betweena seperate thread of execution and callbacks
 */


PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
{
    //Subscribing to odometry
    sub1_ = nh_.subscribe("robot_0/odom", 1000, &PfmsSample::odomCallback,this);
    //Subscribing to laser
    sub2_ = nh_.subscribe("robot_0/base_scan", 10, &PfmsSample::laserCallback,this);

    //Subscribing to image
    // unlike other topics this requires a image transport rather than simply a node handle
    image_transport::ImageTransport it(nh);
    sub3_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback,this);

    //Publishing an image ... just to show how
    image_pub_ = it_.advertise("test/image", 1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1);
    obstaclecheck_=nh_.advertise<std_msgs::Bool>("obstaclecheck",1);



    //Below is how to get parameters from command line, on command line they need to be _param:=value
    //For example _map_sise:=20.0
    ros::NodeHandle pn("~");
    pn.param<double>("resolution", resolution_, 0.1);
}

PfmsSample::~PfmsSample()
{
//    cv::destroyWindow("view");
}



// A callback for odometry
void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    /**
     * @todo - Ex 1 : Obtain a pose (x,y yaw) from nav_msgs/Odometry
     *
     * - On command line type 'rosmsg show nav_msgs/Odometry'
     * - The position and orientation are in two seperate parts of the message
     * - The orinetation is provided as a quaternion
     * - Which angle to we need?
     * - Ros has a 'tf' library with a helper function to get yaw from the quaternion
     * - http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     * - Consider: Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
     * - Where is time of this message stored
     */
//    ROS_INFO_STREAM("x: " << msg->pose.pose.position.x
//             << ",  y: " << msg->pose.pose.position.y
//             << ",  yaw: "<< tf::getYaw(msg->pose.pose.orientation));


  double x2d=msg->pose.pose.position.x;
  double y2d=msg->pose.pose.position.y;
  double theta=tf::getYaw(msg->pose.pose.orientation);
  pose2d_.x=x2d;
  pose2d_.y=y2d;
  pose2d_.theta=theta;
//  ROS_INFO_STREAM("X is"<<pose2d_.x<<" Y is "<<pose2d_.y<<" theta is "<<pose2d_.theta);



    pose_mtx_.lock();
    pose_=msg->pose.pose;
    pose_mtx_.unlock();

}



void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  /**
   * @todo - Ex 2 : Find the closest point [x,y] to the robot using sensor_msgs::LaserScan
   *
   * - On command line type 'rosmsg show sensor_msgs/LaserScan'
   * - What are we provided in this message?
   * - Do we have the inforamtion in this message to find the closest point?
   * - What part of the message do we need to iterate over?
   * - How do we convert from range data to [x,y] (this is known as polar to cartesian)
   * - https://www.mathsisfun.com/polar-cartesian-coordinates.html
   * - Where is time of this message stored?
   * - Is the closest point identified the same as the one you see as closest on the stage simulator?
   * - Why is this the case?
   */

    double nearest_range = msg->range_max;
    double nearest_range_inten;
    int nearest_index = -1;
    for (int i=0; i<msg->ranges.size(); i++) {
        if (msg->ranges[i] < nearest_range) {
            nearest_range = msg->ranges[i];
            nearest_index = i;
            nearest_range_inten=msg->intensities[i];
        }
    }

    double rangesize=msg->ranges.size();
    double intensize=msg->intensities.size();


    ROS_INFO_STREAM("range is "<<nearest_range<<" intensize is "<< nearest_range_inten);


    double nearest_angle = msg->angle_min + nearest_index * msg->angle_increment;
    double x = nearest_range * cos(nearest_angle);
    double y = nearest_range * sin(nearest_angle);




    double ang= pose2d_.theta+nearest_angle;
    double xl=nearest_range*cos(ang);
    double yl = nearest_range*sin(ang); //local obstacle coordinate (robot as o point
    double nearestdegree= nearest_angle*(180/M_PI);




//     ROS_INFO_STREAM("obstacle is on "<<xl<<" and "<<yl);


//    ROS_INFO_STREAM("Nearest obstacle in laser: " << x << ", " << y);


    double osx=154.5;
    double osy=136.5;
    double resX=0.1;
    double resY=0.1;
    bool obstacledetected=false;
    double xposeonimage;
    double yposeonimage;



    double xg= pose2d_.x+xl;
    double yg=pose2d_.y+yl; // obstacle coordinate on global

    xposeonimage=(pose2d_.x/resX)+osx;
    yposeonimage=osy-(pose2d_.y/resY);   // robot coordinate on image XY


    double xOBonimage=(xg/resX)+osx;;
    double yOBonimage=osy-(yg/resY);   // obstacle coordinate on image XY

//       ROS_INFO_STREAM("the nearest_range is "<< nearest_range);


           cv::Mat map =cv::imread("/home/kuangxianhuang/Desktop/simple_map.png");
           cv::namedWindow("view");
           cv::circle(map,cv::Point(xposeonimage,yposeonimage),5,cv::Scalar(0,0,255));

           double newspeed;
           double  newturnrate;
           bool testing=false;
           xlocal_=xl;
           ylocal_=yl;

            if (nearest_range<=2 && nearest_range_inten==1){
              ROS_INFO_STREAM("closing obstacle"<< " at "<< xg <<" and "<<yg);

                testing = true;

                ROS_INFO_STREAM(" ++++ Having obstacile in front ++++ " <<testing);
                cv::circle(map,cv::Point(xOBonimage,yOBonimage),5,cv::Scalar(255,0,0));
//                newspeed=0;
//                newturnrate=0.0;
            }
            else{
              bool testing=false;

              cv::circle(map,cv::Point(xOBonimage,yOBonimage),5,cv::Scalar(0,255,0));
//              newspeed=0;
//              newturnrate=0.0;
            }
            testing_=testing;
           cv::imshow("view",map);
           cv::waitKey(1);


            std_msgs::Bool obstaclecheck;
            obstaclecheck.data=testing;
            obstaclecheck_.publish(obstaclecheck);



    point_mtx_.lock();
    nearestPoint_.x=x;nearestPoint_.y=y;
    point_mtx_.unlock();

//    double newspeed=0;
//    double newturnrate=0;
//    geometry_msgs::Twist control;          // container for the control system
//    control.linear.x = newspeed;        // linear velocity of the control system
//    control.angular.z = newturnrate;      // angular velocity of the control system
//    cmd_vel_pub_.publish(control);      // publishing the control system

}


void PfmsSample::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //! Below code converts from sensor message to a pointer to an opencv image and time to a deque, to share across threads
    try
    {
      if (enc::isColor(msg->encoding))
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
      else
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //To obtain image we use belows
   img_mtx_.lock();
   image_= cvPtr_->image;//! We take the image out of CvPtr and store it in image_ variable (copy);
    /**
     * Ex 3 : Find the closest occupied point [x,y] to the robot using the cv::Mat image
     * The code was folded into Ex 5
     *
     * - What information do we convert from the OgMap pixels to [x,y] : ANSWER - resolution
     * - Where is 0,0 on the image? : ANSWER - top right corner
     *  o------> j
     *  |
     *  |
     *  v i
     * - Where is (0,0) on the OgMap? : ANSWER - top left corner
     */

cv::Mat image2=cv_bridge::toCvShare(msg, "bgr8")->image;

double ximage=(xlocal_/0.1)+250;
double yimage=250-(ylocal_/0.1);
cv::circle(image2,cv::Point(ximage,yimage),5,cv::Scalar(0,255,0));







      if (testing_==true )
      {

       ROS_INFO_STREAM("testing_ = "<<testing_);

        cv::circle(image2,cv::Point(ximage,yimage),5,cv::Scalar(255,0,0));
        cv::imshow("ogmap", image2);

        cv::namedWindow("ogmap");

      }
      else {
        ROS_INFO_STREAM("testing_ = "<<testing_);

         cv::circle(image2,cv::Point(ximage,yimage),5,cv::Scalar(0,255,0));
      }





   img_mtx_.unlock();






}


void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    */

    cv_bridge::CvImage cv_image; // This cv_bridge object is used for publishing images in ROS

    //! What does this rate limiter do?
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) {

      /**
       * @todo Ex 4 : Find the closest point [x,y] to the robot using the robot pose and laser data
       *
       * - If we need to combine the information and use it in this seperate thread what do we need to consider
       * - Is the closest point identified the same as the one you see as closest on the stage simulator?
       * - Why is this the case?
       */
      //In the below we use a special mutex that allows trying to lock for a certain amount of time and then giving up
      //This allows us to avoid deadlock, and still keep the thraed running within some guarantees of max run-time
      bool have_pose=false,have_point=false;
      geometry_msgs::Pose pose;
      geometry_msgs::Point pt;
      if (pose_mtx_.try_lock_for(std::chrono::milliseconds(5))) { //This tries to mutex lock for 5ms and then gives up
            pose=pose_;
            pose_mtx_.unlock();
            have_pose=true;
      }

      if (point_mtx_.try_lock_for(std::chrono::milliseconds(5))) {//This tries to mutex lock for 5ms and then gives up
            pt=nearestPoint_;
            point_mtx_.unlock();
            have_point=true;
      }

      if(have_pose && have_point){ //If we have both nearest point and pose we can compute it in global.
        double yaw = tf::getYaw(pose.orientation);
        //We need to transform closest point to global reference frame.
        // Easiest is to be in polar coordinates
        double d = pow(pow(pt.x,2)+pow(pt.y,2),0.5); // This is how far the point is in Local frame
        double theta = atan2(pt.y,pt.x); // This is the angle to the point in Local frame
        theta+=yaw; // The robot has a yaw, so let's adjust for this
        pt.x=d*cos(theta); //This is new computed point x with angle adjusted
        pt.y=d*sin(theta); //This is new computed point y with angle adjusted

        //We need to shift the point by the robot location to get into global coordinates;
        pt.x+=pose.position.x; // Now the point is in global coordinates
        pt.y+=pose.position.y; // Now the point is in global coordinates

        //If your more advertures and for 3D - 41013 Robotics is the subject dealing with Robot Maths
//        ROS_INFO_STREAM("GLOBAL Nearest obstacle: [x,y]=[" << pt.x << ", " << pt.y << "]");
      }

      //For the image we can lock mutex and then check if image is empty
      //By doing so, we will not get into a position whereby this thread get's to an empty image first (no OgMap
      //received at this stage, and we could segfault accessing memory that is not available as yet.
      img_mtx_.lock();
      if(!image_.empty()){

        /**
        * This was intially in section for Ex 3
        * Moved here for Ex5 - Find the closest occupied point [x,y] to the robot using the cv::Mat image
        *
        * - Where is time of this message stored?
        * - What information do we convert from the OgMap pixels to [x,y] : ANSWER - resolution
        * - Where is 0,0 on the image? : ANSWER - top left corner
        *  o------> j (pt.x)
        *  |
        *  |
        *  v i (pt.y)
        * - Where is (0,0) on the OgMap? : ANSWER - top left corner
        */

        //Loop examining row and col values
        double closest_dist=pow(image_.rows,2)+pow(image_.cols,2); //definately will be less than this.
        cv::Point closest_point(0,0);

        for (int i = 0; i<image_.rows; i++) {
          for (int j = 0; j <image_.cols; j++) {
              unsigned char &pixel = image_.at<unsigned char>(i, j);
              if (pixel == 0) {
               double d = pow( pow( (i-(image_.rows/2))  ,2) + pow( (j-(image_.cols/2)) ,2),0.5);
               if (d<closest_dist){
                 closest_dist=d;
                 closest_point.x=j;closest_point.y=i;
               }
              }
          }
        }
        closest_dist*=resolution_;

        /**
        * Ex 5 : Mark the closest point on a RGB version of the OgMap image
        *
        * - In this seperate thread we only use the OgMap, stored as image_, and use a mutex for access
        * - The closest point identified is th same as seen on the stage simulator as the map follws the vehicle and
        * does not change with robot orientation
        */


        //Below code takes the cv::Mat image which is single channel (grayscale) and converts it into a RGB image
        cv::cvtColor(image_,cv_image.image, CV_GRAY2RGB);
        img_mtx_.unlock(); //We can unlock here having finished with image_

//        ROS_INFO_STREAM("Closest occupied point is:" << closest_dist << " [m] away");

        // draw a circle at closest point of size 3 pixels and in green color
        cv::circle(cv_image.image, closest_point, 3, CV_RGB(0,255, 0) , 1);

        // To publish - send the image we need to specify the encoding and make a header
        cv_image.encoding = "bgr8";
        cv_image.header = std_msgs::Header();
        // We now publish the image and use the inbuilt ros function to convert cv_image to a image message
        image_pub_.publish(cv_image.toImageMsg());
      }
      else{
        img_mtx_.unlock();
      }
      rate_limiter.sleep();
    }

}

