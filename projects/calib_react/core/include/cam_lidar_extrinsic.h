/**
 * @file cam_lidar_calib_node.cpp
 * @author eunsoo
 * @brief customizing cam_lidar_calib package
 * @version 0.1
 * @date 2022-05-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <calibration_error_term.h>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <string.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
// #include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <random>
#include <string>

#include "ceres/ceres.h"
#include "ceres/covariance.h"
#include "ceres/rotation.h"
#include "opencv2/opencv.hpp"
#include "pcl/io/pcd_io.h"
// TODO : rosbag, file 구분하여 처리, projection 코드 확인

enum POINT_DIRECTION {
  X_,
  Y_,
  Z_,
};
namespace Eigen {
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

class camLidarCalib {
 private:
  cv::Mat projection_matrix_;
  cv::Mat distCoeff_;
  std::vector<cv::Point2f> image_points_;
  std::vector<cv::Point3f> object_points_;
  std::vector<cv::Point2f> projected_points_;
  bool boardDetectedInCam_;
  double dx_, dy_;
  int checkerboard_rows_, checkerboard_cols_;
  int min_points_on_plane_ = 500;
  cv::Mat tvec_, rvec_;
  cv::Mat C_R_W_;
  Eigen::Matrix3d c_R_w_;
  Eigen::Vector3d c_t_w_;
  Eigen::Vector3d r3_;
  Eigen::Vector3d r3_old_;
  Eigen::Vector3d Nc_;

  // bool filtering_ = true;
  // std::string load_img_path_, load_pcd_path_;

  // std::string file_type_;
  // std::vector<open3d::geometry::PointCloud> all_points_;
  int num_views_ = 9;
  int image_width_, image_height_;
  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;
  double ransac_threshold_ = 0.01;

  bool checkerboard_set_ = false;
  bool camera_param_set_ = false;

 public:
  std::vector<Eigen::Vector3d> lidar_points_;
  std::vector<std::vector<Eigen::Vector3d>> all_lidar_points_;
  std::vector<Eigen::Vector3d> all_normals_;
  Eigen::Matrix3d rotation_matrix_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_vector_ = Eigen::Vector3d::Zero();
  // std::vector<cv::Mat> images_;
  std::map<std::string, cv::Mat> images_;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> pcds_;
  uchar channel_ = 3;
  // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcds_;
  // ImagemConverter Base64_converter_;
  int readCheckerboardParams(double& dx, double& dy, int& rows, int& cols);
  int readCameraParams(double* camera_info);
  int readCheckerPosition(double* filter_info);

  bool imageHandler(const cv::Mat& image);
  bool cloudHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud);
  bool solveWithPCL();

  camLidarCalib() {
    // getParamFunc();
    c_R_w_ = Eigen::Matrix3d::Zero();
    c_t_w_ = Eigen::Vector3d::Zero();
    r3_ = Eigen::Vector3d::Zero();
    r3_old_ = Eigen::Vector3d::Zero();
    Nc_ = Eigen::Vector3d::Zero();

    projection_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    distCoeff_ = cv::Mat::zeros(5, 1, CV_64F);
    boardDetectedInCam_ = false;
    tvec_ = cv::Mat::zeros(3, 1, CV_64F);
    rvec_ = cv::Mat::zeros(3, 1, CV_64F);
    C_R_W_ = cv::Mat::eye(3, 3, CV_64F);
    c_R_w_ = Eigen::Matrix3d::Identity();
  }
  ~camLidarCalib();
};

camLidarCalib::~camLidarCalib() {
  std::cout << "terminate node" << std::endl;
}
int camLidarCalib::readCameraParams(double* camera_info) {
  image_height_ = (int)camera_info[0];
  image_width_ = (int)camera_info[1];
  distCoeff_.at<double>(0) = camera_info[2];
  distCoeff_.at<double>(1) = camera_info[3];
  distCoeff_.at<double>(2) = camera_info[4];
  distCoeff_.at<double>(3) = camera_info[5];
  distCoeff_.at<double>(4) = camera_info[6];
  projection_matrix_.at<double>(0, 0) = camera_info[7];
  projection_matrix_.at<double>(1, 1) = camera_info[8];
  projection_matrix_.at<double>(0, 2) = camera_info[9];
  projection_matrix_.at<double>(1, 2) = camera_info[10];
  projection_matrix_.at<double>(2, 2) = 1;
  camera_param_set_ = true;
  std::cout << "intinsic : " << std::endl
            << projection_matrix_ << std::endl;
  std::cout << "distCoeff_ : " << std::endl
            << distCoeff_ << std::endl;
  std::cout << "image_width_ : " << std::endl
            << image_width_ << std::endl;
  std::cout << "image_height_ : " << std::endl
            << image_height_ << std::endl;
  return 1;
}

int camLidarCalib::readCheckerboardParams(double& dx, double& dy, int& rows, int& cols) {
  checkerboard_rows_ = rows;
  checkerboard_cols_ = cols;
  dx_ = dx;
  dy_ = dy;
  checkerboard_set_ = true;
  for (int i = 0; i < checkerboard_rows_; i++)
    for (int j = 0; j < checkerboard_cols_; j++)
      object_points_.emplace_back(cv::Point3f(j * dy_, i * dx_, 0.0));
  std::cout << dx_ << std::endl
            << dy_ << std::endl
            << checkerboard_rows_ << std::endl
            << checkerboard_cols_ << std::endl;
  return 1;
}
int camLidarCalib::readCheckerPosition(double* filter_info) {
  x_min_ = filter_info[0];
  x_max_ = filter_info[1];
  y_min_ = filter_info[2];
  y_max_ = filter_info[3];
  z_min_ = filter_info[4];
  z_max_ = filter_info[5];
  std::cout << x_min_ << std::endl
            << x_max_ << std::endl
            << y_min_ << std::endl
            << y_max_ << std::endl
            << z_min_ << std::endl
            << z_max_ << std::endl;
  return 1;
}
bool camLidarCalib::imageHandler(const cv::Mat& image) {
  // image_points_.clear();
  // std::vector<cv::Point2f>().swap(image_points_);
  if (-1 == cv::findChessboardCorners(image, cv::Size(checkerboard_cols_, checkerboard_rows_), image_points_, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE)) {
    return false;
  }
  // cv::drawChessboardCorners(image, cv::Size(checkerboard_cols_, checkerboard_rows_), image_points_, boardDetectedInCam_);
  // std::cout << image_points_.size() << "==" << object_points_.size() << std::endl;
  // if (image_points_.size() == object_points_.size()) {
  cv::solvePnP(object_points_, image_points_, projection_matrix_, distCoeff_, rvec_, tvec_, false);
  projected_points_.clear();
  cv::projectPoints(object_points_, rvec_, tvec_, projection_matrix_, distCoeff_, projected_points_, cv::noArray());
  // std::cout << projected_points_.size() << std::endl;
  // for (int i = 0; i < projected_points_.size(); i++) {
  //     cv::circle(image, projected_points_[i], 16, cv::Scalar(0, 255, 0), 10, cv::LINE_AA, 0);
  // }
  cv::Rodrigues(rvec_, C_R_W_);
  cv::cv2eigen(C_R_W_, c_R_w_);
  c_t_w_ = Eigen::Vector3d(tvec_.at<double>(0), tvec_.at<double>(1), tvec_.at<double>(2));
  r3_ = c_R_w_.block<3, 1>(0, 2);
  Nc_ = (r3_.dot(c_t_w_)) * r3_;
  all_normals_.push_back(Nc_);
  // } else {
  //     std::cout << "image proessing error : num is not matched" << std::endl;
  //     return false;
  // }
  return true;
}
bool camLidarCalib::cloudHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud) {
  std::cout << in_cloud->points.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  /// Pass through filters
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(in_cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_min_, x_max_);
  pass_x.filter(*cloud_filtered_x);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud_filtered_x);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_min_, y_max_);
  pass_y.filter(*cloud_filtered_y);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloud_filtered_y);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_min_, z_max_);
  pass_z.filter(*cloud_filtered_z);
  /// Plane Segmentation
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_filtered_z));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(ransac_threshold_);
  ransac.computeModel();
  std::vector<int> inliers_indicies;
  ransac.getInliers(inliers_indicies);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud_filtered_z, inliers_indicies, *plane);

  /// Statistical Outlier Removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(plane);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1);
  sor.filter(*plane_filtered);

  /// Store the points lying in the filtered plane in a vector
  lidar_points_.clear();
  for (size_t i = 0; i < plane_filtered->points.size(); i++) {
    double X = plane_filtered->points[i].x;
    double Y = plane_filtered->points[i].y;
    double Z = plane_filtered->points[i].z;
    lidar_points_.push_back(Eigen::Vector3d(X, Y, Z));
  }

  // ROS_INFO_STREAM("No of planar_pts: " << lidar_points.size());
  std::cout << "No of planar_pts: " << plane_filtered->points.size() << std::endl;
  if (plane_filtered->points.size() < min_points_on_plane_) return false;
  all_lidar_points_.push_back(lidar_points_);

  return true;
}

bool camLidarCalib::solveWithPCL() {
  // if (r3_.dot(r3_old_) > 0.95) {
  //     return false;
  // }

  // r3_old_ = r3_;
  // all_normals_.push_back(Nc_);
  // all_lidar_points_.push_back(lidar_points_);

  // Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  rotation_matrix_ << 0, -1, 0, 0, 0, -1, 1, 0, 0;
  // Eigen::Vector3d translation_vector = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis_angle;
  ceres::RotationMatrixToAngleAxis(rotation_matrix_.data(), axis_angle.data());  //  convert to Rodrigus type

  Eigen::Vector6d opt_param;

  opt_param(0) = axis_angle(0);
  opt_param(1) = axis_angle(1);
  opt_param(2) = axis_angle(2);
  opt_param(3) = translation_vector_(0);
  opt_param(4) = translation_vector_(1);
  opt_param(5) = translation_vector_(2);

  ceres::LossFunction* loss_function = NULL;
  ceres::Problem problem;
  problem.AddParameterBlock(opt_param.data(), opt_param.size());

  for (int i = 0; i < (int)all_normals_.size(); i++) {
    Eigen::Vector3d normal_i = all_normals_[i];
    std::vector<Eigen::Vector3d> lidar_point_i = all_lidar_points_[i];
    for (int j = 0; j < lidar_point_i.size(); j++) {
      Eigen::Vector3d lidar_point = lidar_point_i[j];
      ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CalibrationErrorTerm, 1, 6>(new CalibrationErrorTerm(lidar_point, normal_i));
      problem.AddResidualBlock(cost_function, loss_function, opt_param.data());
    }
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  // options.preconditioner_type = ceres::SCHUR_JACOBI;

  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  // options.linear_solver_type = ceres::DENSE_SCHUR;

  options.minimizer_progress_to_stdout = true;
  // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  axis_angle(0) = opt_param(0);
  axis_angle(1) = opt_param(1);
  axis_angle(2) = opt_param(2);
  ceres::AngleAxisToRotationMatrix(axis_angle.data(), rotation_matrix_.data());
  translation_vector_(0) = opt_param(3);
  translation_vector_(1) = opt_param(4);
  translation_vector_(2) = opt_param(5);
  std::cout << rotation_matrix_ << std::endl;
  std::cout << translation_vector_ << std::endl;
}