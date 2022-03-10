/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pcl/point_types.h>
//#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

#ifdef USE_PCL_OPENMP

#include <pclomp/ndt_omp.h>

#endif

#include <time.h>

struct pose {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

enum class MethodType {
  PCL_GENERIC = 0,
  PCL_ANH = 1,
  PCL_ANH_GPU = 2,
  PCL_OPENMP = 3,
};

std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
    {"KDTREE", pclomp::KDTREE},
    {"DIRECT7", pclomp::DIRECT7},
    {"DIRECT1", pclomp::DIRECT1}
};

static MethodType _method_type = MethodType::PCL_GENERIC;
static int search_method = 0;
static int thread_num = -1;

struct PointXYZIRPYT {
  PCL_ADD_POINT4D

  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll,
                                                                                                       roll)(float,
                                                                                                             pitch,
                                                                                                             pitch)(
                                      float, yaw, yaw)(double, time, time))

// global variables
static pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom, current_pose,
    current_pose_imu, current_pose_odom, current_pose_imu_odom, ndt_pose, added_pose;//, localizer_pose;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
    offset_imu_odom_yaw;

static double current_velocity_x = 0.0;
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

static pcl::PointCloud<pcl::PointXYZI> map;
std::vector<pcl::PointCloud<pcl::PointXYZI>> keyframes;
int keyframe_id;
pcl::PointCloud<pcl::PointXYZI> keyframes_xyz;
pcl::PointCloud<PointXYZIRPYT> keyframes_xyzrpy;

//surround keyframes
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree;
double surround_radius;
std::vector<int> surround_frame_idx;
std::vector<float> surround_frame_dist;
pcl::PointCloud<pcl::PointXYZI> surround_keyframes;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
static cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;
#ifdef CUDA_FOUND
static gpu::GNormalDistributionsTransform anh_gpu_ndt;
#endif
#ifdef USE_PCL_OPENMP
static pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> omp_ndt;
#endif

double yaw_factor = 1.0;
double accX_factor = 1.0;

// Default values
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.3;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 0.2;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher map_cloud_pub, test_cloud_pub, keyframes_xyz_pub, surround_keyframes_pub;
static ros::Publisher current_pose_pub;
static ros::Publisher guess_pose_linaer_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 0.55;
static double max_scan_range = 200.0;
static double min_add_scan_shift = 0.5;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol;//, tf_ltob;

static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;

static bool _incremental_voxel_update = false;

static std::string _imu_topic = "/imu_raw";
static std::string _points_topic = "/points_raw";
static std::string _odom_topic = "/vehicle/odom";

static double fitness_score;
static bool has_converged;
static int final_num_iteration;
static double transformation_probability;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;

static std::ofstream ofs;
static std::string filename;

static tf::StampedTransform base2odom_transform, odom2map_transform;
pthread_mutex_t mutex;

static void imu_odom_calc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;

  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

  previous_time = current_time;
}

static void odom_calc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose.x + offset_odom_x;
  guess_pose_odom.y = previous_pose.y + offset_odom_y;
  guess_pose_odom.z = previous_pose.z + offset_odom_z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

  previous_time = current_time;
}

static void imu_calc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
      std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
      std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  guess_pose_imu.x = previous_pose.x + offset_imu_x;
  guess_pose_imu.y = previous_pose.y + offset_imu_y;
  guess_pose_imu.z = previous_pose.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

  previous_time = current_time;
}

static double wrapToPm(double a_num, const double a_max) {
  if (a_num >= a_max) {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(double a_angle_rad) {
  return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad) {
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

static void odom_callback(const nav_msgs::Odometry::ConstPtr &input) {
  // std::cout << __func__ << std::endl;

  odom = *input;
  odom_calc(input->header.stamp);
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input) {
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

static void imu_callback(const sensor_msgs::Imu::Ptr &input) {
  // std::cout << __func__ << std::endl;

  if (_imu_upside_down)
    imuUpsideDown(input);

  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();
//    std::cout << "diff_time:" << diff_time << std::endl;

  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  ////////////////////////////////
  imu_roll = 0.0 * imu_roll;
  imu_pitch = 0.0 * imu_pitch;
  imu_yaw = yaw_factor * imu_yaw;  ///-1
  /////////////////////////////////

  imu_roll = wrapToPmPi(imu_roll);
  imu_pitch = wrapToPmPi(imu_pitch);
  imu_yaw = wrapToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  ////////////////////////////////
  imu.linear_acceleration.x = accX_factor * imu.linear_acceleration.x;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;
  /////////////////////////////////

  if (diff_time != 0) {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
//        std::cout << "imu.angular_velocity.x:" << imu.angular_velocity.x << std::endl;
//        std::cout << "imu.angular_velocity.y:" << imu.angular_velocity.y << std::endl;
//        std::cout << "imu.angular_velocity.z:" << imu.angular_velocity.z << std::endl;
//        std::cout << "input->angular_velocity.x:" << input->angular_velocity.x << std::endl;
//        std::cout << "input->angular_velocity.y:" << input->angular_velocity.y << std::endl;
//        std::cout << "input->angular_velocity.z:" << input->angular_velocity.z << std::endl;
//        imu.angular_velocity.x = input->angular_velocity.x;
//        imu.angular_velocity.y = input->angular_velocity.y;
//        imu.angular_velocity.z = input->angular_velocity.z;

  } else {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}

static void exact_surround_keyframes() {
  pcl::PointXYZI curr_pose;
  curr_pose.x = current_pose.x;
  curr_pose.y = current_pose.y;
  curr_pose.z = current_pose.z;

  kdtree->setInputCloud(keyframes_xyz.makeShared());

  std::cout << "Neighbors within radius search at (" << curr_pose.x
            << " " << curr_pose.y
            << " " << curr_pose.z
            << ") with radius=" << surround_radius << std::endl;

  if (kdtree->radiusSearch(curr_pose, surround_radius, surround_frame_idx, surround_frame_dist) > 0) {
    surround_keyframes.clear();
    for (std::size_t i = 0; i < surround_frame_idx.size(); i++) {
      surround_keyframes += keyframes[surround_frame_idx[i]];
      std::cout << "    id:" << i << "\t" << keyframes_xyz.points[surround_frame_idx[i]].x
                << " " << keyframes_xyz.points[surround_frame_idx[i]].y
                << " " << keyframes_xyz.points[surround_frame_idx[i]].z
                << " (squared distance: " << surround_frame_dist[i] << ")" << std::endl;
    }

    //publish surround keyframes
    sensor_msgs::PointCloud2::Ptr surround_keyframes_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(surround_keyframes, *surround_keyframes_msg_ptr);
    surround_keyframes_pub.publish(*surround_keyframes_msg_ptr);
  }

}

static void map_pub_callback(const ros::TimerEvent &e) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  //publish map when new frame added
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_cloud_pub.publish(*map_msg_ptr);
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

//    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  //transform scan to base_link first
  pcl::transformPointCloud(tmp, *transformed_scan_ptr, tf_btol);
  transformed_scan_ptr->header.frame_id = "base_link";

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = (*transformed_scan_ptr).begin();
       item != (*transformed_scan_ptr).end(); item++) {
    p.x = (double) item->x;
    p.y = (double) item->y;
    p.z = (double) item->z;
    p.intensity = (double) item->intensity;

    // minmax
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range) {
      scan.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

//    pcl::PointCloud<pcl::PointXYZI>::Ptr pre_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  // Apply RadiusOutlierRemoval filter
//    pcl::RadiusOutlierRemoval<pcl::PointXYZI>::Ptr rad(new pcl::RadiusOutlierRemoval<pcl::PointXYZI>());
//    rad->setRadiusSearch(0.20);
//    rad->setMinNeighborsInRadius(3);
//    rad->setInputCloud(scan_ptr);
//    rad->filter(*pre_filtered_ptr);

//    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//    sor.setInputCloud(scan_ptr);
//    sor.setMeanK(10);
//    sor.setStddevMulThresh(1.0);
//    sor.filter(*pre_filtered_ptr);

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0) {
    map += *filtered_scan_ptr;

    //load first keyframe
    keyframe_id = 0;
    keyframes.push_back(*filtered_scan_ptr);
//        keyframes_xyz_.push_back(pcl::PointXYZI(keyframe_id_));

    pcl::PointXYZI p_3d;
    PointXYZIRPYT p_6d;
    p_3d.x = p_6d.x = current_pose.x;
    p_3d.y = p_6d.y = current_pose.y;
    p_3d.z = p_6d.z = current_pose.z;
    p_3d.intensity = p_6d.intensity = keyframe_id;

    p_6d.roll = current_pose.roll;
    p_6d.pitch = current_pose.pitch;
    p_6d.yaw = current_pose.yaw;
    p_6d.time = current_scan_time.toSec();
    keyframes_xyz.push_back(p_3d);
    keyframes_xyzrpy.push_back(p_6d);

    initial_scan_loaded = 1;
  }

  //setInputSource
  if (_method_type == MethodType::PCL_GENERIC) {
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setStepSize(step_size);
    ndt.setResolution(ndt_res);
    ndt.setMaximumIterations(max_iter);
    ndt.setInputSource(filtered_scan_ptr);
  } else if (_method_type == MethodType::PCL_ANH) {
    anh_ndt.setTransformationEpsilon(trans_eps);
    anh_ndt.setStepSize(step_size);
    anh_ndt.setResolution(ndt_res);
    anh_ndt.setMaximumIterations(max_iter);
    anh_ndt.setInputSource(filtered_scan_ptr);
  }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
    {
      anh_gpu_ndt.setTransformationEpsilon(trans_eps);
      anh_gpu_ndt.setStepSize(step_size);
      anh_gpu_ndt.setResolution(ndt_res);
      anh_gpu_ndt.setMaximumIterations(max_iter);
      anh_gpu_ndt.setInputSource(filtered_scan_ptr);
    }
#endif
#ifdef USE_PCL_OPENMP
  else if (_method_type == MethodType::PCL_OPENMP) {
    omp_ndt.setTransformationEpsilon(trans_eps);
    omp_ndt.setStepSize(step_size);
    omp_ndt.setResolution(ndt_res);
    omp_ndt.setMaximumIterations(max_iter);
    omp_ndt.setInputSource(filtered_scan_ptr);
    omp_ndt.setNeighborhoodSearchMethod(search_methods[search_method].second);
    if (thread_num < 1 || thread_num > omp_get_max_threads())
      omp_ndt.setNumThreads(omp_get_max_threads());
    else
      omp_ndt.setNumThreads(thread_num);
  }
#endif

  static bool is_first_map = true;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  if (is_first_map == true) {
    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setInputTarget(map_ptr);
    else if (_method_type == MethodType::PCL_ANH)
      anh_ndt.setInputTarget(map_ptr);
#ifdef CUDA_FOUND
      else if (_method_type == MethodType::PCL_ANH_GPU)
        anh_gpu_ndt.setInputTarget(map_ptr);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputTarget(map_ptr);
#endif
    is_first_map = false;
  }

  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  if (_use_imu == true && _use_odom == true)
    imu_odom_calc(current_scan_time);
  if (_use_imu == true && _use_odom == false)
    imu_calc(current_scan_time);
  if (_use_imu == false && _use_odom == true)
    odom_calc(current_scan_time);

  pose guess_pose_for_ndt;
  if (_use_imu == true && _use_odom == true)
    guess_pose_for_ndt = guess_pose_imu_odom;
  else if (_use_imu == true && _use_odom == false)
    guess_pose_for_ndt = guess_pose_imu;
  else if (_use_imu == false && _use_odom == true)
    guess_pose_for_ndt = guess_pose_odom;
  else
    guess_pose_for_ndt = guess_pose;

  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();// * tf_btol;

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  t4_start = ros::Time::now();

  std::chrono::time_point<std::chrono::system_clock> align_start, align_end;
  static double align_time = 0.0;
//  static double total_align_time = 0.0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (_method_type == MethodType::PCL_GENERIC) {
    align_start = std::chrono::system_clock::now();
    ndt.align(*output_cloud, init_guess);
    align_end = std::chrono::system_clock::now();

    fitness_score = ndt.getFitnessScore();
    t_base_link = ndt.getFinalTransformation();
    has_converged = ndt.hasConverged();
    final_num_iteration = ndt.getFinalNumIteration();
    transformation_probability = ndt.getTransformationProbability();
  } else if (_method_type == MethodType::PCL_ANH) {
    align_start = std::chrono::system_clock::now();
    anh_ndt.align(init_guess);
    align_end = std::chrono::system_clock::now();

    fitness_score = anh_ndt.getFitnessScore();
    t_base_link = anh_ndt.getFinalTransformation();
    has_converged = anh_ndt.hasConverged();
    final_num_iteration = anh_ndt.getFinalNumIteration();
  }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
    {
      align_start = std::chrono::system_clock::now();
      anh_gpu_ndt.align(init_guess);
      align_end = std::chrono::system_clock::now();

      fitness_score = anh_gpu_ndt.getFitnessScore();
      t_base_link = anh_gpu_ndt.getFinalTransformation();
      has_converged = anh_gpu_ndt.hasConverged();
      final_num_iteration = anh_gpu_ndt.getFinalNumIteration();
    }
#endif
#ifdef USE_PCL_OPENMP
  else if (_method_type == MethodType::PCL_OPENMP) {
    align_start = std::chrono::system_clock::now();
    omp_ndt.align(*output_cloud, init_guess);
    align_end = std::chrono::system_clock::now();
    fitness_score = omp_ndt.getFitnessScore();
    t_base_link = omp_ndt.getFinalTransformation();
    has_converged = omp_ndt.hasConverged();
    final_num_iteration = omp_ndt.getFinalNumIteration();
  }
#endif

  align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
//  total_align_time += align_time;

  //trans the scan to map frame
  pcl::transformPointCloud(*filtered_scan_ptr, *transformed_scan_ptr, t_base_link);

  // Update ndt_pose and current_pose.
  tf::Matrix3x3 mat_b;//orientation matrix
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;

  //broadcast current base_link tf
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  pthread_mutex_lock(&mutex);
  odom2map_transform =
      tf::StampedTransform(transform * base2odom_transform.inverse(), current_scan_time, "/map", "/odom");
  pthread_mutex_unlock(&mutex);

  br.sendTransform(odom2map_transform);
//  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  // publish current_pose
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();
  current_pose_pub.publish(current_pose_msg);


  //smooth
  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;

  current_pose_imu.x = current_pose.x;
  current_pose_imu.y = current_pose.y;
  current_pose_imu.z = current_pose.z;
  current_pose_imu.roll = current_pose.roll;
  current_pose_imu.pitch = current_pose.pitch;
  current_pose_imu.yaw = current_pose.yaw;

  current_pose_odom.x = current_pose.x;
  current_pose_odom.y = current_pose.y;
  current_pose_odom.z = current_pose.z;
  current_pose_odom.roll = current_pose.roll;
  current_pose_odom.pitch = current_pose.pitch;
  current_pose_odom.yaw = current_pose.yaw;

  current_pose_imu_odom.x = current_pose.x;
  current_pose_imu_odom.y = current_pose.y;
  current_pose_imu_odom.z = current_pose.z;
  current_pose_imu_odom.roll = current_pose.roll;
  current_pose_imu_odom.pitch = current_pose.pitch;
  current_pose_imu_odom.yaw = current_pose.yaw;

  current_velocity_imu_x = current_velocity_x;
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;


  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift) {
    map += *transformed_scan_ptr;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;

    keyframe_id += 1;
    keyframes.push_back(*transformed_scan_ptr);
    pcl::PointXYZI p_3d;
    PointXYZIRPYT p_6d;
    p_3d.x = p_6d.x = current_pose.x;
    p_3d.y = p_6d.y = current_pose.y;
    p_3d.z = p_6d.z = current_pose.z;
    p_3d.intensity = p_6d.intensity = keyframe_id;

    p_6d.roll = current_pose.roll;
    p_6d.pitch = current_pose.pitch;
    p_6d.yaw = current_pose.yaw;
    p_6d.time = current_pose_msg.header.stamp.toSec();
    keyframes_xyz.push_back(p_3d);
    keyframes_xyzrpy.push_back(p_6d);

    //publish keyframes position
    sensor_msgs::PointCloud2::Ptr keyframes_xyz_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(keyframes_xyz, *keyframes_xyz_msg_ptr);
    keyframes_xyz_pub.publish(*keyframes_xyz_msg_ptr);

    exact_surround_keyframes();
    pcl::PointCloud<pcl::PointXYZI>::Ptr surround_map_ptr(new pcl::PointCloud<pcl::PointXYZI>(surround_keyframes));

    //set target map
    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setInputTarget(surround_map_ptr);
    else if (_method_type == MethodType::PCL_ANH) {
      if (_incremental_voxel_update == true)
        anh_ndt.updateVoxelGrid(transformed_scan_ptr);
      else
        anh_ndt.setInputTarget(surround_map_ptr);
    }
#ifdef CUDA_FOUND
      else if (_method_type == MethodType::PCL_ANH_GPU)
        anh_gpu_ndt.setInputTarget(surround_map_ptr);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputTarget(surround_map_ptr);
#endif

//        //publish map when new frame added
//        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
//        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
//        map_cloud_pub.publish(*map_msg_ptr);
  }


  // Write log
  if (!ofs) {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << input->header.seq << ","
      << input->header.stamp << ","
      << input->header.frame_id << ","
      << scan_ptr->size() << ","
      << filtered_scan_ptr->size() << ","
      << std::fixed << std::setprecision(5) << current_pose.x << ","
      << std::fixed << std::setprecision(5) << current_pose.y << ","
      << std::fixed << std::setprecision(5) << current_pose.z << ","
      << current_pose.roll << ","
      << current_pose.pitch << ","
      << current_pose.yaw << ","
      << final_num_iteration << ","
      << fitness_score << ","
      << ndt_res << ","
      << step_size << ","
      << trans_eps << ","
      << max_iter << ","
      << voxel_leaf_size << ","
      << min_scan_range << ","
      << max_scan_range << ","
      << min_add_scan_shift << std::endl;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
//    std::cout << "Number of pre filtered scan points: " << pre_filtered_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "Number of map Points: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_base_link << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "Align time: " << align_time << std::endl;
//  std::cout << "Total align time: " << total_align_time << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void *thread_func(void *args) {

  ros::NodeHandle nh_tf_listener;
  tf::TransformListener tf_listener;
  ros::Rate ros_rate(100);


  while (nh_tf_listener.ok()) {
    try {

      static tf::StampedTransform new_base2odom_transform;
      tf_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.2));
      tf_listener.lookupTransform("/odom", "/base_link", ros::Time(0), new_base2odom_transform);

      pthread_mutex_lock(&mutex);
      base2odom_transform = new_base2odom_transform;
      pthread_mutex_unlock(&mutex);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }

    ros_rate.sleep();

  }

  return nullptr;
}

int main(int argc, char **argv) {
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  current_pose_imu.x = 0.0;
  current_pose_imu.y = 0.0;
  current_pose_imu.z = 0.0;
  current_pose_imu.roll = 0.0;
  current_pose_imu.pitch = 0.0;
  current_pose_imu.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  ros::init(argc, argv, "ndt_mapping");
  pthread_mutex_init(&mutex, NULL);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm *pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  filename = "ndt_mapping_" + std::string(buffer) + ".csv";
  ofs.open(filename.c_str(), std::ios::app);

  // write header for log file
  if (!ofs) {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << "input->header.seq" << ","
      << "input->header.stamp" << ","
      << "input->header.frame_id" << ","
      << "scan_ptr->size()" << ","
      << "filtered_scan_ptr->size()" << ","
      << "current_pose.x" << ","
      << "current_pose.y" << ","
      << "current_pose.z" << ","
      << "current_pose.roll" << ","
      << "current_pose.pitch" << ","
      << "current_pose.yaw" << ","
      << "final_num_iteration" << ","
      << "fitness_score" << ","
      << "ndt_res" << ","
      << "step_size" << ","
      << "trans_eps" << ","
      << "max_iter" << ","
      << "voxel_leaf_size" << ","
      << "min_scan_range" << ","
      << "max_scan_range" << ","
      << "min_add_scan_shift" << std::endl;

  // setting parameters
  int method_type_tmp = 0;
  private_nh.getParam("method_type", method_type_tmp);
  _method_type = static_cast<MethodType>(method_type_tmp);
  private_nh.getParam("imu_upside_down", _imu_upside_down);
  private_nh.getParam("imu_topic", _imu_topic);
  private_nh.getParam("odom_topic", _odom_topic);
  private_nh.getParam("points_topic", _points_topic);
  private_nh.getParam("incremental_voxel_update", _incremental_voxel_update);

  private_nh.getParam("max_iter", max_iter);
  private_nh.getParam("ndt_res", ndt_res);
  private_nh.getParam("step_size", step_size);
  private_nh.getParam("trans_eps", trans_eps);
  private_nh.getParam("search_method", search_method);
  private_nh.getParam("thread_num", thread_num);

  private_nh.getParam("voxel_leaf_size", voxel_leaf_size);
  private_nh.getParam("min_scan_range", min_scan_range);
  private_nh.getParam("max_scan_range", max_scan_range);
  private_nh.getParam("min_add_scan_shift", min_add_scan_shift);
  private_nh.param("surround_radius", surround_radius, 20.0);

  private_nh.getParam("yaw_factor", yaw_factor);
  private_nh.getParam("accX_factor", accX_factor);

  std::cout << "method_type: " << static_cast<int>(_method_type) << std::endl;
  std::cout << "thread_num: " << thread_num << std::endl;
  std::cout << "search_method: " << search_methods[search_method].first << std::endl;
  std::cout << "use_odom: " << _use_odom << std::endl;
  std::cout << "use_imu: " << _use_imu << std::endl;
  std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
  std::cout << "imu_topic: " << _imu_topic << std::endl;
  std::cout << "odom_topic: " << _odom_topic << std::endl;
  std::cout << "points_topic: " << _points_topic << std::endl;

  std::cout << "incremental_voxel_update: " << _incremental_voxel_update << std::endl;

  if (private_nh.getParam("tf_x", _tf_x) == false) {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_y", _tf_y) == false) {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_z", _tf_z) == false) {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_roll", _tf_roll) == false) {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_pitch", _tf_pitch) == false) {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (private_nh.getParam("tf_yaw", _tf_yaw) == false) {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

#ifndef CUDA_FOUND
  if (_method_type == MethodType::PCL_ANH_GPU) {
    std::cerr << "**************************************************************" << std::endl;
    std::cerr << "[ERROR]PCL_ANH_GPU is not built. Please use other method type." << std::endl;
    std::cerr << "**************************************************************" << std::endl;
    exit(1);
  }
#endif
#ifndef USE_PCL_OPENMP
  if (_method_type == MethodType::PCL_OPENMP)
  {
    std::cerr << "**************************************************************" << std::endl;
    std::cerr << "[ERROR]PCL_OPENMP is not built. Please use other method type." << std::endl;
    std::cerr << "**************************************************************" << std::endl;
    exit(1);
  }
#endif

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
//    tf_ltob = tf_btol.inverse();

  map.header.frame_id = "map";
  keyframes_xyz.header.frame_id = "map";
  keyframes_xyzrpy.header.frame_id = "map";
  surround_keyframes.header.frame_id = "map";
  kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 10);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
  test_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud", 1000);
  keyframes_xyz_pub = nh.advertise<sensor_msgs::PointCloud2>("/keyframes_xyz", 1000);
  surround_keyframes_pub = nh.advertise<sensor_msgs::PointCloud2>("/surround_keyframes", 1000);

  ros::Subscriber points_sub = nh.subscribe(_points_topic, 10, points_callback);
  ros::Subscriber odom_sub = nh.subscribe(_odom_topic, 100, odom_callback);
  ros::Subscriber imu_sub = nh.subscribe(_imu_topic, 100, imu_callback);

  ros::Timer map_pub_timer = nh.createTimer(ros::Duration(3.0), map_pub_callback);

  pthread_t thread;
  pthread_create(&thread, NULL, thread_func, NULL);

  ros::spin();

  return 0;
}
