//#PLACEHOLDER  FOR COPYRIGHT SHIT

#ifndef ORBSLAM2_ROS_MONONODE_H_
#define ORBSLAM2_ROS_MONONODE_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
// Odometry subscriber
#include <nav_msgs/Odometry.h>

#include "System.h"
#include "Node.h"


class MonoNode : public Node
{
  public:
    MonoNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~MonoNode ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msg);
    void OdomCallback(const nav_msgs::Odometry::Ptr& msg);
    unsigned int frame;

  private:
    image_transport::Subscriber image_subscriber;
    ros::Subscriber odom_subscriber;
    nav_msgs::Odometry::Ptr odom_ptr;
};

#endif //ORBSLAM2_ROS_MONONODE_H_
