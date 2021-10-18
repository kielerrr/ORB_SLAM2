/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <dynamic_reconfigure/server.h>
#include <orb_slam2_ros/dynamic_reconfigureConfig.h>

#include "orb_slam2_ros/SaveMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
//*
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// Odometry subscriber
#include <nav_msgs/Odometry.h>

#include "System.h"



class Node
{
  public:
    Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();
    void Init ();

  protected:
    void Update ();
    void UpdateAlt(unsigned int frame_cntr);
    //* Added methods
    std::string getTestStr(int test_id);
    double UpdateScaleFactor (const nav_msgs::Odometry::ConstPtr& odom);
    void ScaleMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points, double s);
    void ekfPredict ();
    void ekfCorrect ();
    std::vector< std::vector<double> > ReadDB(std::string filename);
    std::vector<double> GetBoundingBox(std::vector<double> det);
    std::vector< std::vector<double> > abvol;
    //*
    ORB_SLAM2::System* orb_slam_;
    ros::Time current_frame_time_;
    ros::Time past_frame_time_;
    cv::Mat curr_position;
    cv::Mat prev_position;
    //* Added variables
    bool init_scale_;
    bool load_file_;
    bool write_file_;
    bool object_detection_enabled;
    int frame_cntr;
    std::vector< std::vector<double> > dset_obj_;       // Dataset of detected objects  
    std::vector< cv::Point3f > object_points_;
    int obj_cntr_;                                      // Counter for objects in dataset
    int n_objects_;
    std::string filename_;
    std::string filepath_;
    std::string outfilename_;
    std::string outpath_;
    int test_id_;
    std::string test_str_;
    //*
    std::string camera_info_topic_;

  private:
    //* Alternative object point cloud publisher 
    void PublishAltMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublishObjectPoints (std::vector<cv::Point3f> &object_points);
    void PublishBoundingVol (std::vector< std::vector<double> > &bvol);
    void PrintBoundingVol(std::vector< std::vector<double> > &bvol);
    void writeOD(std::vector< std::vector<double> > &bvol);
    void AddNewBoundingVol(std::vector< std::vector<double> > &bvol, std::vector<int> &lbl, unsigned int frame, std::vector< std::vector<double> > &abvol);
    sensor_msgs::PointCloud2 ObjectPointsToPointCloud (std::vector<cv::Point3f> &object_points);
    sensor_msgs::PointCloud2 TrackedMapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> tmap_points);
    ros::Publisher object_points_publisher_;
    ros::Publisher tracked_map_points_publisher_;
    ros::Publisher marker_pub_;
    //*
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishRenderedImage (cv::Mat image);
    void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level);
    bool SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);
    void LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters);

    tf::Transform TransformFromMat (cv::Mat position_mat);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);

    dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;

    ros::ServiceServer service_server_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ORB_SLAM2::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    std::string drone_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;
};

#endif //ORBSLAM2_ROS_NODE_H_
