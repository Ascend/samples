//
// Created by ascend on 2021/5/13.
//

#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <velodyne_pcl/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

#ifdef USE_PCL_OPENMP

#include <pclomp/ndt_omp.h>

#endif

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <std_srvs/Empty.h>
#include<random>
#include "lidar_localizer/GlobalLocalizer.h"

bool serviceStarted = false;
bool relocalizing = false;
bool map_loaded = false;
bool lidar_loaded = false;
bool init_pos_set = false;
int bestPoseId = -1;

int InnerPointsNum = 20;
int MatchNum = 10;

float fitnessScore;
ros::Publisher initial_pose_pub;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol;

static std::string _localizer = "rslidar";

static double min_scan_range_ = 0.5;
static double max_scan_range_ = 200;

static double voxel_leaf_size = 0.2;

static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.3;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon
static int thread_num = -1;

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

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
static cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;
#ifdef CUDA_FOUND
static std::shared_ptr<gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr =
    std::make_shared<gpu::GNormalDistributionsTransform>();
#endif
#ifdef USE_PCL_OPENMP
static pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> omp_ndt;
#endif

static pcl::PointCloud<pcl::PointXYZI> map;
pcl::PointCloud<pcl::PointXYZI> filtered_scan;
static unsigned int points_map_num = 0;

static bool has_converged;
static int iteration = 0;
static double bestFitnessScore = 10000.0;
static double trans_probability = 0.0;

pthread_mutex_t mutex;

struct pose {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct point2d {
  float x;
  float y;
};

struct InitPose {
  float x;
  float y;
  float yaw;
};

static std::vector<point2d> pointList = {
    point2d{0.224329635501, -0.0982430130243},
    point2d{-0.147755116224, 0.0762454271317},
    point2d{-0.835154891014, 0.147877082229},
    point2d{-1.69589877129, 0.285447716713}};

std::vector<InitPose> generatePose() {

  std::vector<InitPose> poseList;
  point2d a, b;

  for (int pointId = 0; pointId < pointList.size() - 1; pointId++) {
    a = pointList[pointId];
    b = pointList[pointId + 1];

    double dist_x = b.x - a.x;
    double dist_y = b.y - a.y;

    for (int i = 0; i < InnerPointsNum; i++) {
      InitPose pose;
      pose.x = a.x + dist_x * i / InnerPointsNum;
      pose.y = a.y + dist_y * i / InnerPointsNum;

      ROS_INFO("generating..... :(%f,%f)", pose.x, pose.y);

      for (double yaw = -3.14; yaw <= 3.14; yaw += 0.0436) {
        pose.yaw = yaw;
        poseList.push_back(pose);
      }
    }
  }

  return poseList;
}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {

  if (points_map_num != input->width) {
    ROS_INFO("Update points_map.");

    points_map_num = input->width;

    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

    // Setting point cloud to be aligned to.
    if (_method_type == MethodType::PCL_GENERIC) {
      pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> new_ndt;
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      new_ndt.setResolution(ndt_res);
      new_ndt.setInputTarget(map_ptr);
      new_ndt.setMaximumIterations(max_iter);
      new_ndt.setStepSize(step_size);
      new_ndt.setTransformationEpsilon(trans_eps);

      new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      ndt = new_ndt;
      pthread_mutex_unlock(&mutex);
    } else if (_method_type == MethodType::PCL_ANH) {
      cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> new_anh_ndt;
      new_anh_ndt.setResolution(ndt_res);
      new_anh_ndt.setInputTarget(map_ptr);
      new_anh_ndt.setMaximumIterations(max_iter);
      new_anh_ndt.setStepSize(step_size);
      new_anh_ndt.setTransformationEpsilon(trans_eps);

      pcl::PointCloud<pcl::PointXYZI>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointXYZI dummy_point;
      dummy_scan_ptr->push_back(dummy_point);
      new_anh_ndt.setInputSource(dummy_scan_ptr);

      new_anh_ndt.align(Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      anh_ndt = new_anh_ndt;
      pthread_mutex_unlock(&mutex);
    }
#ifdef CUDA_FOUND
      else if (_method_type == MethodType::PCL_ANH_GPU)
      {
        std::shared_ptr<gpu::GNormalDistributionsTransform> new_anh_gpu_ndt_ptr =
            std::make_shared<gpu::GNormalDistributionsTransform>();
        new_anh_gpu_ndt_ptr->setResolution(ndt_res);
        new_anh_gpu_ndt_ptr->setInputTarget(map_ptr);
        new_anh_gpu_ndt_ptr->setMaximumIterations(max_iter);
        new_anh_gpu_ndt_ptr->setStepSize(step_size);
        new_anh_gpu_ndt_ptr->setTransformationEpsilon(trans_eps);

        pcl::PointCloud<pcl::PointXYZI>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointXYZI dummy_point;
        dummy_scan_ptr->push_back(dummy_point);
        new_anh_gpu_ndt_ptr->setInputSource(dummy_scan_ptr);

        new_anh_gpu_ndt_ptr->align(Eigen::Matrix4f::Identity());

        pthread_mutex_lock(&mutex);
        anh_gpu_ndt_ptr = new_anh_gpu_ndt_ptr;
        pthread_mutex_unlock(&mutex);
      }
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP) {
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> new_omp_ndt;
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      new_omp_ndt.setResolution(ndt_res);
      new_omp_ndt.setInputTarget(map_ptr);
      new_omp_ndt.setMaximumIterations(max_iter);
      new_omp_ndt.setStepSize(step_size);
      new_omp_ndt.setTransformationEpsilon(trans_eps);
      new_omp_ndt.setNeighborhoodSearchMethod(search_methods[search_method].second);

      if (thread_num < 1 || thread_num > omp_get_max_threads())
        new_omp_ndt.setNumThreads(omp_get_max_threads());
      else
        new_omp_ndt.setNumThreads(thread_num);

      new_omp_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      omp_ndt = new_omp_ndt;
      pthread_mutex_unlock(&mutex);
    }
#endif
    map_loaded = true;
    ROS_INFO("cloud map loaded......");
  }
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {

//  ROS_INFO("lidar point cloud loaded......");

  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan, transformed_scan;

  ros::Time current_scan_time = input->header.stamp;
  static ros::Time previous_scan_time = current_scan_time;

  pcl::fromROSMsg(*input, tmp);

  //transform scan to base_link first
  pcl::transformPointCloud(tmp, transformed_scan, tf_btol);
  transformed_scan.header.frame_id = "base_link";

  double r;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = transformed_scan.begin();
       item != transformed_scan.end(); item++) {

    p.x = (double) item->x;
    p.y = (double) item->y;
    p.z = (double) item->z;

    p.intensity = (double) item->intensity;

    // minmax
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range_ < r && r < max_scan_range_) {
      scan.push_back(p);
    }
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(filtered_scan);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filtered_scan));
  int scan_points_num = filtered_scan_ptr->size();

  pthread_mutex_lock(&mutex);
  if (_method_type == MethodType::PCL_GENERIC)
    ndt.setInputSource(filtered_scan_ptr);
  else if (_method_type == MethodType::PCL_ANH)
    anh_ndt.setInputSource(filtered_scan_ptr);
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt_ptr->setInputSource(filtered_scan_ptr);
#endif
#ifdef USE_PCL_OPENMP
  else if (_method_type == MethodType::PCL_OPENMP)
    omp_ndt.setInputSource(filtered_scan_ptr);
#endif
  pthread_mutex_unlock(&mutex);

  lidar_loaded = true;
}

bool globalLocalizeCb(lidar_localizer::GlobalLocalizer::Request &req, lidar_localizer::GlobalLocalizer::Response &res) {

  if (map_loaded == false || lidar_loaded == false || relocalizing == true) {

    if (relocalizing == true) {
      ROS_INFO("global_relocalizer is running .......");
    } else {
      ROS_INFO("relocalizer waitting for map or lidar .......");
    }

    res.feedback = "false";
    return false;
  }

  relocalizing = true;
  ROS_INFO("global relocalizing .......");

  std::vector<InitPose> poseList;
  poseList = generatePose();

  for (int poseId = 0; poseId < poseList.size(); poseId++) {
    float sum_fitness_score = 0.0, average_fitness_score = 0.0;
    bool average_fitness_score_updated = false;
    for (int matchId = 0; matchId < MatchNum; matchId++) {
      /////////////////match once////////////////////
      static float fitness_score = 0.0;

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      tf::Quaternion predict_q, ndt_q, current_q, localizer_q;

      Eigen::Matrix4f t_b(Eigen::Matrix4f::Identity());   // base_link
//        Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());  // localizer

      std::chrono::time_point<std::chrono::system_clock> align_start, align_end,
          getFitnessScore_start, getFitnessScore_end;
      static double align_time, getFitnessScore_time = 0.0;
      static double total_align_time = 0.0;

      pose predict_pose_for_ndt;
      predict_pose_for_ndt.x = poseList[poseId].x;
      predict_pose_for_ndt.y = poseList[poseId].y;
      predict_pose_for_ndt.z = 0.0f;
      predict_pose_for_ndt.roll = 0.0f;
      predict_pose_for_ndt.pitch = 0.0f;
      predict_pose_for_ndt.yaw = poseList[poseId].yaw;

      Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
      Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

      Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y *
          init_rotation_x).matrix();// * tf_btol;

      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

      if (_method_type == MethodType::PCL_GENERIC) {
        align_start = std::chrono::system_clock::now();
        ndt.align(*output_cloud, init_guess);
        align_end = std::chrono::system_clock::now();

        has_converged = ndt.hasConverged();

        t_b = ndt.getFinalTransformation();
        iteration = ndt.getFinalNumIteration();

        getFitnessScore_start = std::chrono::system_clock::now();
        fitness_score = ndt.getFitnessScore();
        getFitnessScore_end = std::chrono::system_clock::now();

        trans_probability = ndt.getTransformationProbability();
      } else if (_method_type == MethodType::PCL_ANH) {
        align_start = std::chrono::system_clock::now();
        anh_ndt.align(init_guess);
        align_end = std::chrono::system_clock::now();

        has_converged = anh_ndt.hasConverged();

        t_b = anh_ndt.getFinalTransformation();
        iteration = anh_ndt.getFinalNumIteration();

        getFitnessScore_start = std::chrono::system_clock::now();
        fitness_score = anh_ndt.getFitnessScore();
        getFitnessScore_end = std::chrono::system_clock::now();

        trans_probability = anh_ndt.getTransformationProbability();
      }
#ifdef CUDA_FOUND
        else if (_method_type == MethodType::PCL_ANH_GPU)
    {
      align_start = std::chrono::system_clock::now();
      anh_gpu_ndt_ptr->align(init_guess);
      align_end = std::chrono::system_clock::now();

      has_converged = anh_gpu_ndt_ptr->hasConverged();

      t = anh_gpu_ndt_ptr->getFinalTransformation();
      iteration = anh_gpu_ndt_ptr->getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      fitness_score = anh_gpu_ndt_ptr->getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      trans_probability = anh_gpu_ndt_ptr->getTransformationProbability();
    }
#endif
#ifdef USE_PCL_OPENMP
      else if (_method_type == MethodType::PCL_OPENMP) {
        align_start = std::chrono::system_clock::now();
        omp_ndt.align(*output_cloud, init_guess);
        align_end = std::chrono::system_clock::now();

        has_converged = omp_ndt.hasConverged();

        t_b = omp_ndt.getFinalTransformation();
        iteration = omp_ndt.getFinalNumIteration();

        getFitnessScore_start = std::chrono::system_clock::now();
        fitness_score = omp_ndt.getFitnessScore();
        getFitnessScore_end = std::chrono::system_clock::now();

        trans_probability = omp_ndt.getTransformationProbability();
      }
#endif
      /////////////////match once////////////////////

      if (fitness_score > 0.01) break;
      sum_fitness_score += fitness_score;
      if (matchId == MatchNum - 1) {
        average_fitness_score = sum_fitness_score / MatchNum;
        average_fitness_score_updated = true;
      }
    }

    if (average_fitness_score_updated) {
      if (average_fitness_score < bestFitnessScore) {
        bestFitnessScore = average_fitness_score;
        bestPoseId = poseId;

        ROS_INFO("best iteration: (%d)", iteration);
        ROS_INFO("best fitness score: (%f)", bestFitnessScore);
        ROS_INFO("best fitness id: (%d)", poseId);
        ROS_INFO("best x, y, yaw: (%f,%f,%f)",
                 poseList[bestPoseId].x,
                 poseList[bestPoseId].y,
                 poseList[bestPoseId].yaw);

        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = poseList[bestPoseId].x;
        pose_msg.pose.pose.position.y = poseList[bestPoseId].y;
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.covariance[0] = 0.25;
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25;
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942;
        pose_msg.pose.pose.orientation.x = 0.0;
        pose_msg.pose.pose.orientation.y = 0.0;
        pose_msg.pose.pose.orientation.z = sin(poseList[bestPoseId].yaw / 2);
        pose_msg.pose.pose.orientation.w = cos(poseList[bestPoseId].yaw / 2);

        initial_pose_pub.publish(pose_msg);
        ROS_INFO("(global localizer) Setting initial pose to x,y,yaw,z,w :(%f,%f,%f,%f,%f)",
                 poseList[bestPoseId].x,
                 poseList[bestPoseId].y,
                 poseList[bestPoseId].yaw,
                 sin(poseList[bestPoseId].yaw / 2),
                 cos(poseList[bestPoseId].yaw / 2));

      }
    }
  }
//    // for example current posee
//
//    default_random_engine e(time(0));
//    uniform_real_distribution<double> u(-3.0, 3.0);
//    for (int i = 0; i < 10; ++i)
//        cout << u(e) << endl;

  res.feedback = "true";
  relocalizing = false;
  return true;
}

static void fitnessCb(const std_msgs::Float32 fitnessScore_) {
  fitnessScore = fitnessScore_.data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_localizer_server");
  pthread_mutex_init(&mutex, NULL);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int method_type_tmp = 0;
  private_nh.getParam("method_type", method_type_tmp);
  _method_type = static_cast<MethodType>(method_type_tmp);
  private_nh.getParam("voxel_leaf_size", voxel_leaf_size);
  private_nh.getParam("max_iter", max_iter);
  private_nh.getParam("ndt_res", ndt_res);
  private_nh.getParam("trans_eps", trans_eps);
  private_nh.getParam("search_method", search_method);
  private_nh.getParam("thread_num", thread_num);

  private_nh.getParam("min_scan_range", min_scan_range_);
  private_nh.getParam("max_scan_range", max_scan_range_);

  if (private_nh.getParam("localizer", _localizer) == false) {
    std::cout << "localizer is not set." << std::endl;
    return 1;
  }
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

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

//  ros::Subscriber fitness_sub = nh.subscribe("/ndt_fitness_score", 10, fitnessCb);
  ros::Subscriber map_sub = nh.subscribe("map_cloud", 10, map_callback);
  ros::Subscriber points_sub = nh.subscribe("/points_raw", 10, points_callback);
  initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

  ros::ServiceServer global_relocalize_server = nh.advertiseService("/global_relocalize", globalLocalizeCb);
  ros::spin();


//  ros::Rate loop_rate(10);
//  while (ros::ok()) {
//    if (map_loaded && lidar_loaded && !serviceStarted) {
//      ros::ServiceServer global_relocalize_server = nh.advertiseService("/global_relocalize", globalLocalizeCb);
//      serviceStarted = true;
//      ROS_INFO("GLOBAL RELOCALIZER SERVICE STARTED!");
//    }
//    ros::spinOnce();
//    loop_rate.sleep();
//  }

}
