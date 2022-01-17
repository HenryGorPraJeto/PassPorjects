#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Pose2D.h"

/**
 * This node shows some connections and publishing images
 */


class RandomWalk{

public:
  /*! @brief RandomWalk constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    RandomWalk(ros::NodeHandle nh);

  /*! @brief RandomWalk destructor.
   *
   *  Will tear down the object
   */
    ~RandomWalk();


 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);


public:
    ros::NodeHandle nh_;

private:
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Publisher cmd_vel_pub_;//! Command velocity publisher

    geometry_msgs::Pose2D pose_;//! 2d pose of the robot
};

