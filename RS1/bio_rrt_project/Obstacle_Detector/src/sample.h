#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Pose2D.h"



namespace enc = sensor_msgs::image_encodings;

/**
 * This node shows some connections and publishing images
 */


class PfmsSample{

public:
  /*! @brief PfmsSample constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    PfmsSample(ros::NodeHandle nh);

  /*! @brief PfmsSample destructor.
   *
   *  Will tear down the object
   */
    ~PfmsSample();

    /*! @brief seperate thread.
     *
     *  The main processing thread that will run continously and utilise the data
     *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
     */

      void seperateThread();

private:

  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

 /*! @brief ImageConst Callback
   *
   *  @param sensor_msgs::ImageConstPtr - The imageconst message
   *  @note This function and the declaration are ROS specific
   */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);


public:
    ros::NodeHandle nh_;

private:
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    image_transport::Subscriber sub3_;
    cv_bridge::CvImagePtr cvPtr_;
    ros::ServiceServer service_;
//    ros::Subscriber sub4_;
    ros::Publisher cmd_vel_pub_;//! Command velocity publisher
    ros::Publisher obstaclecheck_;

    double xlocal_;
    double ylocal_;
    double nearest_range_;
    bool testing_;


    double resolution_;//! size of OgMap in pixels

    //Refer Tutorial week 7 Ex 3 for a more elegant way to group data and mutex/convars
    cv::Mat image_; //! The OgMap as a image
    std::mutex img_mtx_;//! Mutex to control access to image

    geometry_msgs::Pose pose_; //! Pose of the robot
    geometry_msgs::Pose2D pose2d_;
    std::timed_mutex pose_mtx_;//! Mutex to control access to pose

    geometry_msgs::Point nearestPoint_; //! Nearest point as determined by laser
    std::timed_mutex point_mtx_;//! Mutex to control access to nearest point

};

