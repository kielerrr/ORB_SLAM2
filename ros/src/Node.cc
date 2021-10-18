#include "Node.h"

#include <iostream>

Node::Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) :  image_transport_(image_transport) {
  name_of_node_ = ros::this_node::getName();
  node_handle_ = node_handle;
  min_observations_per_point_ = 2;
  sensor_ = sensor;
}


Node::~Node () {
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  if(object_detection_enabled && write_file_){
    writeOD(abvol);
  }

  delete orb_slam_;
}

void Node::Init () {
  //static parameters
  node_handle_.param(name_of_node_+ "/publish_pointcloud", publish_pointcloud_param_, true);
  node_handle_.param(name_of_node_+ "/publish_pose", publish_pose_param_, true);
  node_handle_.param(name_of_node_+ "/publish_tf", publish_tf_param_, true);
  node_handle_.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
  node_handle_.param<std::string>(name_of_node_+ "/camera_frame_id", camera_frame_id_param_, "camera_link");
  node_handle_.param<std::string>(name_of_node_ + "/map_file", map_file_name_param_, "map.bin");
  node_handle_.param<std::string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
  node_handle_.param(name_of_node_ + "/load_map", load_map_param_, false);
  node_handle_.param(name_of_node_ + "/test_id", test_id_, 0);
  node_handle_.param<std::string>(name_of_node_ + "/drone_name", drone_name_param_, "");
   // Create a parameters object to pass to the Tracking system
   ORB_SLAM2::ORBParameters parameters;
   LoadOrbParameters (parameters);

  orb_slam_ = new ORB_SLAM2::System (voc_file_name_param_, sensor_, parameters, map_file_name_param_, load_map_param_);

  service_server_ = node_handle_.advertiseService(name_of_node_+"/save_map", &Node::SaveMapSrv, this);

  //Setup dynamic reconfigure
  dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
  dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  dynamic_param_server_.setCallback(dynamic_param_callback);

  rendered_image_publisher_ = image_transport_.advertise (name_of_node_+"/debug_image", 1);
  if (publish_pointcloud_param_) {
    map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_ + "/map_points", 1);
    tracked_map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_ + "/tracked_map_points", 1);
    object_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_ + "/object_points_scaled", 1);
    marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1);
  }

  //* 
  //frame_cntr = 0;
  object_detection_enabled = true; // Switch between normal/object-detection mode 

  test_str_= getTestStr(test_id_);
  obj_cntr_ = 0;
  init_scale_ = false;
  load_file_ = true;
  write_file_ = true;
  filename_ = drone_name_param_+ "_object_detection_"+test_str_+".csv";
  filepath_ = "/home/victor/amaius_scripts/output_files/" + filename_;
  outfilename_ = drone_name_param_ + "_bounding_volumes_XX.csv";
  outpath_ = "/home/victor/Desktop/amaius_scripts/output_files/" + outfilename_;
  
  cout << filepath_ << std::endl;
  

  if(object_detection_enabled){ //* Get object detection data
    if(load_file_){
      std::cout << "Loaded found objects database" << std::endl;
      dset_obj_ = ReadDB(filepath_);
      n_objects_ = dset_obj_.size();
    }
  }
}


void Node::Update() {
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    if (publish_tf_param_){
      PublishPositionAsTransform(position);
    }

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped(position);
    }
  }

  PublishRenderedImage (orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetAllMapPoints());
  }
}

void Node::UpdateAlt(unsigned int frame_cntr) {
  std::vector< std::vector<double> > det_obj_;       // Set of detected objects in current frame
  std::vector<int> det_obj_lbl_;                     // Set of detected objects labels
  std::vector< std::vector<double> > bvol;           // Set of bounding volumes from detected objects
  bool object_detection_ = false;
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    PublishPositionAsTransform (position);

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped (position);
    }
  }
  if(object_detection_enabled){
    //* Check for detected objects in current frame.
    bool multiple_objects_;
    do{
      multiple_objects_ = false;
      if(frame_cntr - 1 == dset_obj_.at(obj_cntr_).at(6)){
        std:vector<double> bbox = GetBoundingBox(dset_obj_.at(obj_cntr_));
        det_obj_lbl_.push_back(dset_obj_.at(obj_cntr_).at(1));
        det_obj_.push_back(bbox);
        // Check for multiple objects in same frame
        if(obj_cntr_ == n_objects_){
          multiple_objects_ = false;
        }
        else{
          if(dset_obj_.at(obj_cntr_).at(6) == dset_obj_.at(obj_cntr_ + 1).at(6)){
            multiple_objects_ = true;
          }
          obj_cntr_ ++;  
        }
        object_detection_ = true;
      }
    } while(multiple_objects_); 
  }
  //
  if(object_detection_enabled){
    PublishRenderedImage (orb_slam_->DrawCurrentFrameAlt(object_detection_, det_obj_, bvol));  
  }
  else{
    PublishRenderedImage (orb_slam_->DrawCurrentFrame());
  }

  if (publish_pointcloud_param_) {
    //* Scale point cloud, by factor s
    //ScaleMapPoints(orb_slam_->GetAllMapPoints(), float s=2.5);

    PublishMapPoints (orb_slam_->GetAllMapPoints());
    //*
    if(object_detection_enabled){
      if(object_detection_){
        PublishBoundingVol(bvol);
        AddNewBoundingVol(bvol, det_obj_lbl_, frame_cntr, abvol);
        //PrintBoundingVol(bvol);
      }
    }
  }
  std::cout << "frame: " << frame_cntr << std::endl;
}


double Node::UpdateScaleFactor (const nav_msgs::Odometry::ConstPtr& odom) {
  // Method deprecated 
  double s, d = -1., d_e = -1.;
  current_frame_time_ = ros::Time::now();
  curr_position = orb_slam_->GetCurrentPosition();
  if (!init_scale_) {
    init_scale_ = true;
    prev_position = curr_position;
    past_frame_time_ = current_frame_time_;
  } else {
    
    if (!curr_position.empty()) {
      ros::Duration time_diff = current_frame_time_- past_frame_time_;
      double dt = time_diff.toSec();

      // Expected displacement (from odometry)
      double dx = odom->twist.twist.linear.x * dt;
      double dy = odom->twist.twist.linear.y * dt;
      double dz = odom->twist.twist.linear.z * dt;
      d = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
      
      if (curr_position.rows == prev_position.rows) {
        // Compute displacement between current and previous pose
        cv::Mat pose_diff;
        cv::absdiff(curr_position, prev_position, pose_diff);
        // Extract translation from pose and convert to robot coordinates  z -> y (front), -y -> z (up)
        double dx_e = pose_diff.at<float>(0, 3);      // x <= -x
        double dy_e = pose_diff.at<float>(1, 3);      // y <= z
        double dz_e = pose_diff.at<float>(2, 3);      // z <= -y
        // cout << "dx_e: " << dx_e << "dy_e: " << dy_e << "dz_e: " << dz_e << endl;
        d_e = sqrt(pow(dx_e, 2) + pow(dy_e, 2) + pow(dz_e, 2));
      }
      
      if (d > 1e-5 && d_e > 1e-5){
        s = d/d_e;
      } else {
        s = 1.;
      }
      cout << "Odom distance: " << d << endl;
      cout << "Estimated distance: " << d_e << endl;
      cout << "Scale ratio S: " << s << endl;
      prev_position = curr_position;
      past_frame_time_ = current_frame_time_;
    }
  }
  
  
  return s;
}

void Node::ScaleMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points, double s) {
  cout << s << endl;
  for(size_t iMP=0; iMP<map_points.size(); iMP++)
    {
        if(map_points[iMP])
        {
            ORB_SLAM2::MapPoint* pMP = map_points[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*s);
        }
    }
}

std::vector< std::vector<double> > Node::ReadDB(std::string filename){
  std::vector< std::vector<double> > result;
  std::ifstream dbFile(filename);
  if(!dbFile.is_open()) throw std::runtime_error("Could not open file");
  std::string line;
  int val;
  if(dbFile.good())
  {
      // Extract the first line in the file (Column names)
      std::getline(dbFile, line);
  }
  // Read data
  while(std::getline(dbFile, line))
  {
    std::vector<double> r;
    std::string cell;
    std::stringstream lineStream(line);
      while(std::getline(lineStream, cell, ','))
    {
        r.push_back(std::stod(cell));
    }
    result.push_back(r);
  }
  dbFile.close();
  return result;
}

std::vector<double> Node::GetBoundingBox(std::vector<double> det){
  std::vector<double> bbox;
  double scaleU, scaleV;
  if(drone_name_param_=='tello'){
    scaleU = 960./1280;
    scaleV = 720./960;
  }
  if(drone_name_param_=='bebop'){
  scaleU = 856./1280; //960./856;
  scaleV = 480./960; //720./480;
  }
  bbox.push_back(det.at(2) * scaleU); //umin
  bbox.push_back(det.at(3) * scaleV); //vmin
  bbox.push_back((det.at(4)- det.at(2)) * scaleU); //width
  bbox.push_back((det.at(5)- det.at(3)) * scaleV); //height
  return bbox;
}

void Node::PrintBoundingVol(std::vector< std::vector<double> > &bvol){
  for(int j = 0; j < bvol.size(); j++){
    cout << "Object " << j << " bounding volume (min, max): ";
    cout << "x (" << bvol.at(j).at(0) << ", " << bvol.at(j).at(1) << ") ";
    cout << "y (" << bvol.at(j).at(2) << ", " << bvol.at(j).at(3) << ") ";
    cout << "z (" << bvol.at(j).at(4) << ", " << bvol.at(j).at(5) << ")" << endl;
  }
}

void Node::AddNewBoundingVol(std::vector< std::vector<double> > &bvol, std::vector<int> &lbl, unsigned int frame, std::vector< std::vector<double> > &abvol){
  for(int j = 0; j < bvol.size(); j++){
    std::vector<double> r;
    r.push_back(frame);
    r.push_back(lbl.at(j));
    for(int k = 0; k < 6; k++){
      r.push_back(bvol.at(j).at(k));
    }
    abvol.push_back(r);
  }
}

void Node::writeOD(std::vector< std::vector<double> > &bvol){
  std::cout << "Detected object(s) bounding volume(s) stored in: "<< outpath_ << std::endl;
  std::ofstream oFile(outpath_);
  for(int i = 0; i < bvol.size(); i++){
    for(int j = 0; j < bvol.at(0).size(); j++){
      oFile << bvol.at(i).at(j);
      oFile << ",";
    }
    oFile << "\n";
  }
  std::cout << std::endl;
}

void Node::PublishBoundingVol(vector< vector<double> >& bvol){
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "/map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "bounding_volumes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.025;
  // Line list color
  line_list.color.r = 0.0;
  line_list.color.g = 0.0;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p, pmin, pmax;
  for(int i = 0; i < bvol.size(); i++){
    pmin.x = bvol.at(i).at(0), pmin.y = bvol.at(i).at(2), pmin.z = bvol.at(i).at(4);
    line_list.points.push_back(pmin);
    p.x = bvol.at(i).at(1), p.y = bvol.at(i).at(2), p.z = bvol.at(i).at(4);
    line_list.points.push_back(p);
    line_list.points.push_back(pmin);
    p.x = bvol.at(i).at(0), p.y = bvol.at(i).at(3), p.z = bvol.at(i).at(4);
    line_list.points.push_back(p);
    line_list.points.push_back(pmin);
    p.x = bvol.at(i).at(0), p.y = bvol.at(i).at(2), p.z = bvol.at(i).at(5);
    line_list.points.push_back(p);
    //-
    pmax.x = bvol.at(i).at(1), pmax.y = bvol.at(i).at(3), pmax.z = bvol.at(i).at(5);
    line_list.points.push_back(pmax);
    p.x = bvol.at(i).at(0), p.y = bvol.at(i).at(3), p.z = bvol.at(i).at(5);
    line_list.points.push_back(p);
    line_list.points.push_back(pmax);
    p.x = bvol.at(i).at(1), p.y = bvol.at(i).at(2), p.z = bvol.at(i).at(5);
    line_list.points.push_back(p);
    line_list.points.push_back(pmax);
    p.x = bvol.at(i).at(1), p.y = bvol.at(i).at(3), p.z = bvol.at(i).at(4);
    line_list.points.push_back(p);
    //- 3
    pmin.x = bvol.at(i).at(0), pmin.y = bvol.at(i).at(3), pmin.z = bvol.at(i).at(5);
    line_list.points.push_back(pmin);
    p.x = bvol.at(i).at(0), p.y = bvol.at(i).at(3), p.z = bvol.at(i).at(4);
    line_list.points.push_back(p);
    line_list.points.push_back(pmin);
    p.x = bvol.at(i).at(0), p.y = bvol.at(i).at(2), p.z = bvol.at(i).at(5);
    line_list.points.push_back(p);
    //- 4
    pmax.x = bvol.at(i).at(1), pmax.y = bvol.at(i).at(2), pmax.z = bvol.at(i).at(4);
    line_list.points.push_back(pmax);
    p.x = bvol.at(i).at(1), p.y = bvol.at(i).at(3), p.z = bvol.at(i).at(4);
    line_list.points.push_back(p);
    line_list.points.push_back(pmax);
    p.x = bvol.at(i).at(1), p.y = bvol.at(i).at(2), p.z = bvol.at(i).at(5);
    line_list.points.push_back(p);
    //- 5
    pmin.x = bvol.at(i).at(0), pmin.y = bvol.at(i).at(2), pmin.z = bvol.at(i).at(5);
    line_list.points.push_back(pmin);
    pmax.x = bvol.at(i).at(1), pmax.y = bvol.at(i).at(2), pmax.z = bvol.at(i).at(5);
    line_list.points.push_back(pmax);
    //- 6
    pmin.x = bvol.at(i).at(0), pmin.y = bvol.at(i).at(3), pmin.z = bvol.at(i).at(4);
    line_list.points.push_back(pmin);
    pmax.x = bvol.at(i).at(1), pmax.y = bvol.at(i).at(3), pmax.z = bvol.at(i).at(4);
    line_list.points.push_back(pmax);
    // Line list color
    line_list.color.r = 0.0;
    line_list.color.g = 0.0;
    line_list.color.b = 1.0 - i*(0.2);
    line_list.color.a = 1.0;
    marker_pub_.publish(line_list);
  }
}
//*
void Node::PublishObjectPoints (std::vector<cv::Point3f> &object_points) {
  if(!object_points.empty()){
    sensor_msgs::PointCloud2 cloud = ObjectPointsToPointCloud (object_points);
    object_points_publisher_.publish (cloud);
    //std::cout << "Publishing object point cloud"<< std::endl;
  }
}
//*
sensor_msgs::PointCloud2 Node::ObjectPointsToPointCloud (std::vector<cv::Point3f> &object_points) {
  if (object_points.size() == 0) {
    std::cout << "Object point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;
  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = object_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    data_array[0] = object_points.at(i).x; //x. Do the transformation by just reading at the position of z instead of x
    data_array[1] = object_points.at(i).y; //y. Do the transformation by just reading at the position of x instead of y
    data_array[2] = object_points.at(i).z; //z. Do the transformation by just reading at the position of y instead of z

    memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
  }

  return cloud;
}

sensor_msgs::PointCloud2 Node::TrackedMapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> tmap_points) {
  if (tmap_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = tmap_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (tmap_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = tmap_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* tmap_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* tmap_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}

void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if(!map_points.empty()){
    sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
    map_points_publisher_.publish (cloud);  
  }
}

void Node::PublishPositionAsTransform (cv::Mat position) {
  if(publish_tf_param_){
      tf::Transform transform = TransformFromMat (position);
      static tf::TransformBroadcaster tf_broadcaster;
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
  }
}

void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  tf::Transform grasp_tf = TransformFromMat (position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
  pose_publisher_.publish(pose_msg);
}


void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}


tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);


  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}


sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}


void Node::ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level) {
  orb_slam_->EnableLocalizationOnly (config.localize_only);
  min_observations_per_point_ = config.min_observations_for_ros_map;

  if (config.reset_map) {
    orb_slam_->Reset();
    config.reset_map = false;
  }

  orb_slam_->SetMinimumKeyFrames (config.min_num_kf_in_map);
}


bool Node::SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res) {
  res.success = orb_slam_->SaveMap(req.name);

  if (res.success) {
    ROS_INFO_STREAM ("Map was saved as " << req.name);
  } else {
    ROS_ERROR ("Map could not be saved.");
  }

  return res.success;
}


void Node::LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters) {
  //ORB SLAM configuration parameters
  node_handle_.param(name_of_node_ + "/camera_fps", parameters.maxFrames, 30);
  node_handle_.param(name_of_node_ + "/camera_rgb_encoding", parameters.RGB, true);
  node_handle_.param(name_of_node_ + "/ORBextractor/nFeatures", parameters.nFeatures, 1200);
  node_handle_.param(name_of_node_ + "/ORBextractor/scaleFactor", parameters.scaleFactor, static_cast<float>(1.2));
  node_handle_.param(name_of_node_ + "/ORBextractor/nLevels", parameters.nLevels, 8);
  node_handle_.param(name_of_node_ + "/ORBextractor/iniThFAST", parameters.iniThFAST, 20);
  node_handle_.param(name_of_node_ + "/ORBextractor/minThFAST", parameters.minThFAST, 7);

  bool load_calibration_from_cam = false;
  node_handle_.param(name_of_node_ + "/load_calibration_from_cam", load_calibration_from_cam, false);

  if (sensor_== ORB_SLAM2::System::STEREO || sensor_==ORB_SLAM2::System::RGBD) {
    node_handle_.param(name_of_node_ + "/ThDepth", parameters.thDepth, static_cast<float>(35.0));
    node_handle_.param(name_of_node_ + "/depth_map_factor", parameters.depthMapFactor, static_cast<float>(1.0));
  }

  if (load_calibration_from_cam) {
    ROS_INFO_STREAM ("Listening for camera info on topic " << node_handle_.resolveName(camera_info_topic_));
    sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, ros::Duration(1000.0));
    if(camera_info == nullptr){
        ROS_WARN("Did not receive camera info before timeout, defaulting to launch file params.");
    } else {
      parameters.fx = camera_info->K[0];
      parameters.fy = camera_info->K[4];
      parameters.cx = camera_info->K[2];
      parameters.cy = camera_info->K[5];

      parameters.baseline = camera_info->P[3];

      parameters.k1 = camera_info->D[0];
      parameters.k2 = camera_info->D[1];
      parameters.p1 = camera_info->D[2];
      parameters.p2 = camera_info->D[3];
      parameters.k3 = camera_info->D[4];
      return;
    }
  }

  bool got_cam_calibration = true;
  if (sensor_== ORB_SLAM2::System::STEREO || sensor_==ORB_SLAM2::System::RGBD) {
    got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_baseline", parameters.baseline);
  }

  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fx", parameters.fx);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fy", parameters.fy);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cx", parameters.cx);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cy", parameters.cy);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k1", parameters.k1);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k2", parameters.k2);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p1", parameters.p1);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p2", parameters.p2);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k3", parameters.k3);

  if (!got_cam_calibration) {
    ROS_ERROR ("Failed to get camera calibration parameters from the launch file.");
    throw std::runtime_error("No cam calibration");
  }
}

std::string Node::getTestStr(int test_id){
  string test_str;
  if(test_id < 10){
    test_str = "0" + std::to_string(test_id);
  }
  else{
    test_str = std::to_string(test_id);
  }
  return test_str;
}
