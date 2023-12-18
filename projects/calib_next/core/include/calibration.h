/**
 * @file calib.h
 * @author Eunsoo (eslim@superb-ai.com)
 * @brief mono-cam calibration
 * @version 0.1
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/rotation.h>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <opencv2/calib3d/calib3d_c.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <stdio.h>
#include <string.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <random>
#include <string>
#include <vector>

// TODO : roslaunch file and set param
// TODO : stereo inheritance to mono

namespace Eigen {
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}
enum class StereoCam {
  LEFT,
  RIGHT,
};

namespace intrinsic {
class MonoCamCalib {
 private:
  int32_t checkerboard_rows_num, checkerboard_colm_num;
  double checkerboard_dx, checkerboard_dy;  // MEMO . no matter scale
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<cv::Point3f> object;
  cv::Mat intrinsic_param, dist_coeffs;
  std::vector<std::string> file_order;
  cv::Mat rotation_matrix;
  cv::Mat translation_matrix;
  std::map<std::string, cv::Mat> images;

 public:
  explicit MonoCamCalib() : checkerboard_rows_num(0), checkerboard_colm_num(0), checkerboard_dx(0.0), checkerboard_dy(0.0) {
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    rotation_matrix = cv::Mat::zeros(3, 1, CV_64F);
    translation_matrix = cv::Mat::zeros(3, 1, CV_64F);
    intrinsic_param = cv::Mat::zeros(3, 1, CV_64F);
    object_points.clear();
    image_points.clear();
    object.clear();
    images.clear();
    file_order.clear();
    object_points.reserve(30);
    image_points.reserve(30);
    object.reserve(30);
    file_order.reserve(30);
  };
  virtual ~MonoCamCalib() {
    object_points.clear();
    std::vector<std::vector<cv::Point3f>>().swap(object_points);
    image_points.clear();
    std::vector<std::vector<cv::Point2f>>().swap(image_points);
    object.clear();
    std::vector<cv::Point3f>().swap(object);
    file_order.clear();
    std::vector<std::string>().swap(file_order);
    images.clear();
    std::map<std::string, cv::Mat>().swap(images);
  };

  MonoCamCalib(const MonoCamCalib &obj) = delete;
  MonoCamCalib(const MonoCamCalib &&obj) = delete;

  // bool calibRawimage();
  virtual bool preSolveCalib(const std::string &&file_name, std::vector<uchar> &debug_image);
  bool readCheckerboardParams(const double &&dx, const double &&dy, const int &&rows, const int &&cols);
  bool getDecodedImage(const std::string &&file_name, std::vector<uchar> &decoded_image);
  bool removeImage(const std::string &&file_name);
  bool getImageResolution(const std::string &&file_name, std::vector<int32_t> &res) noexcept;
  virtual bool solveCalib(std::vector<double> &result);
  virtual bool readImage(const std::string &&file_name);
  bool clear();
  bool getCalibResult(const std::string &&file_name, std::vector<double> &R_result, std::vector<double> &T_result, std::vector<double> &error_value) noexcept;
};

class StereoCamCalib : public intrinsic::MonoCamCalib {
 private:
  std::map<std::string, cv::Mat> images_left;
  std::map<std::string, cv::Mat> images_right;
  std::vector<std::vector<cv::Point3f>> objpoints_right, objpoints_left;
  std::vector<std::vector<cv::Point2f>> imgpoints_right, imgpoints_left;
  std::vector<cv::Point3f> object;

  cv::Mat cameraMatrix_left, cameraMatrix_right, distCoeffs_left, distCoeffs_right, R_left, R_right, T_left, T_right;
  cv::Mat R1_, R2_, P1_, P2_, Q_;
  cv::Mat rotation_matrix, translation_matrix, essential_matrix, fundamental_matrix;
  // object
  int32_t checkerboard_rows_num, checkerboard_colm_num;
  double checkerboard_dx, checkerboard_dy;  // MEMO . no matter scale
 public:
  virtual bool preSolveCalib(const std::string &&file_name, std::vector<uchar> &debug_image) override;
  bool parsingFilename(const std::string &&file_names, std::vector<std::string> &file_names_vector);
  virtual bool readImage(const std::string &&file_name) override;
  // TODO : implements below function
  virtual bool solveCalib(std::vector<double> &result) override;
  // virtual bool removeImage(const std::string &&file_name) override;

  // virtual bool clear() override;
  explicit StereoCamCalib() = default;
  virtual ~StereoCamCalib();
};

}  // namespace intrinsic

namespace extrinsic {
// enum POINT_DIRECTION {
//   X_,
//   Y_,
//   Z_,
// };
class CamLidarInteraction {
 private:
  /* data */
 public:
  CamLidarInteraction(/* args */);
  ~CamLidarInteraction();
};

class TargetBasedErrorTerm {
 private:
  const Eigen::Vector3d laser_point_;
  const Eigen::Vector3d normal_to_plane_;

 public:
  TargetBasedErrorTerm(const Eigen::Vector3d &laser_point,
                       const Eigen::Vector3d &normal_to_plane) : laser_point_(laser_point), normal_to_plane_(normal_to_plane) {}

  template <typename T>
  bool operator()(const T *const R_t, T *residual) const {
    T l_pt_L[3] = {T(laser_point_(0)), T(laser_point_(1)), T(laser_point_(2))};
    T n_C[3] = {T(normal_to_plane_(0)), T(normal_to_plane_(1)), T(normal_to_plane_(2))};
    T l_pt_C[3];
    ceres::AngleAxisRotatePoint(R_t, l_pt_L, l_pt_C);
    l_pt_C[0] += R_t[3];
    l_pt_C[1] += R_t[4];
    l_pt_C[2] += R_t[5];
    Eigen::Matrix<T, 3, 1> laser_point_C(l_pt_C);
    Eigen::Matrix<T, 3, 1> laser_point_L(l_pt_L);
    Eigen::Matrix<T, 3, 1> normal_C(n_C);
    // projection to camera coordinate  .. compare with camera's norm vector and lidar's normvector
    residual[0] = normal_C.normalized().dot(laser_point_C) - normal_C.norm();
    return true;
  }
};

class CameraLidarCalib {
 private:
  cv::Mat projection_matrix;
  cv::Mat dist_coeffs;
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> projected_points;
  //   bool boardDetectedInCam_;
  double checkerboard_dx, checkerboard_dy;

  int32_t checkerboard_rows, checkerboard_cols;
  int32_t min_points_on_plane;
  cv::Mat cv_translation_vector, cv_rotation_matrix;
  cv::Mat cv_c_r_w;  // cam2world, rotation
  Eigen::Matrix3d c_R_w;
  Eigen::Vector3d c_t_w;
  Eigen::Vector3d r3;
  Eigen::Vector3d r3_old;
  Eigen::Vector3d Nc;

  int32_t image_width, image_height;

  std::unique_ptr<double> x_min;
  std::unique_ptr<double> x_max;
  std::unique_ptr<double> y_min;
  std::unique_ptr<double> y_max;
  std::unique_ptr<double> z_min;
  std::unique_ptr<double> z_max;

  float ransac_threshold;

  uint camera_param_set = 0x0;
  uint checkerboard_param_set = 0x0;
  uint filter_param_set = 0x0;
  uint check_anchor = 0x111;

  std::vector<Eigen::Vector3d> lidar_points;
  std::vector<std::vector<Eigen::Vector3d>> all_lidar_points;
  std::vector<Eigen::Vector3d> all_normals;
  Eigen::Matrix3d rotation_matrix;
  Eigen::Vector3d translation_vector;
  std::map<std::string, cv::Mat> images;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr> pcds;  // MEMO : base type is x,y,z,intensity
  uchar channel;

  bool checkSumToReady() {
    uint check_sum = camera_param_set | checkerboard_param_set | filter_param_set;
    if (check_anchor == check_sum) {
      return true;
    } else {
      return false;
    }
  }

 public:
  bool readImage(const std::string &&file_name);
  bool readPCD(const std::string &&file_name);

  bool readCheckerboardParams(const double &&dx, const double &&dy, const int &&rows, const int &&cols) noexcept;
  bool readCameraParams(const std::vector<double> &&camera_info) noexcept;
  bool readCheckerPosition(const std::vector<double> &&filter_info) noexcept;
  bool imageHandler(const cv::Mat &image);
  bool cloudHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud);
  bool solveWithPCL();
  template <typename T>
  bool getDecodeImage(const std::string &&file_name, std::vector<T> &decoded_data) noexcept;
  template <typename T>
  bool getDecodePCD(const std::string &&file_name, std::vector<T> &decoded_data) noexcept;

  bool preSolveCalibPCD(const std::string &&file_name) noexcept;
  bool preSolveCalibImage(const std::string &&file_name) noexcept;

  bool solveCalib(std::vector<double> &result) noexcept;

  bool getImageResolution(const std::string file_name, std::vector<int32_t> &res);
  bool getPCDSize(const std::string file_name, std::vector<int32_t> &res);
  bool removeImage(const std::string &&file_name);
  bool removePCD(const std::string &&file_name);
  bool clearImage();
  bool clearPCD();

  bool getParam(std::vector<int32_t> &resolution, std::shared_ptr<std::vector<cv::Point2f>> other_image_points, std::shared_ptr<std::vector<cv::Point3f>> other_object_points) {
    // checkerboard.push_back(checkerboard_rows);
    // checkerboard.push_back(checkerboard_cols);
    resolution.push_back(images.begin()->second.cols);
    resolution.push_back(images.begin()->second.rows);
    other_image_points = std::make_shared<std::vector<cv::Point2f>>(image_points);
    other_object_points = std::make_shared<std::vector<cv::Point3f>>(object_points);

    return true;
  }
  // CameraLidarCalib(const CameraLidarCalib &obj) = delete;
  CameraLidarCalib(const CameraLidarCalib &&obj) = delete;
  CameraLidarCalib(const CameraLidarCalib &obj) = delete;

  explicit CameraLidarCalib() : min_points_on_plane(500), ransac_threshold(0.01), channel(4) {
    // TODO: initialization
    // TODO : float -> double
    // TODO: noexcept
    // TODO: pattern : factory method + FACADE
    // TODO : type -> enum

    // TODO :  unique_ptr, shared_ptr [] -> vector
    // TODO : file name print

    rotation_matrix = Eigen::Matrix3d::Identity();
    translation_vector = Eigen::Vector3d::Zero();

    c_R_w = Eigen::Matrix3d::Zero();
    c_t_w = Eigen::Vector3d::Zero();
    r3 = Eigen::Vector3d::Zero();
    r3_old = Eigen::Vector3d::Zero();
    Nc = Eigen::Vector3d::Zero();

    projection_matrix = cv::Mat::zeros(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    cv_translation_vector = cv::Mat::zeros(3, 1, CV_64F);
    cv_rotation_matrix = cv::Mat::zeros(3, 1, CV_64F);
    cv_c_r_w = cv::Mat::eye(3, 3, CV_64F);
    c_R_w = Eigen::Matrix3d::Identity();
  }
  virtual ~CameraLidarCalib() {
    image_points.clear();
    std::vector<cv::Point2f>().swap(image_points);

    object_points.clear();
    std::vector<cv::Point3f>().swap(object_points);

    projected_points.clear();
    std::vector<cv::Point2f>().swap(projected_points);

    lidar_points.clear();
    std::vector<Eigen::Vector3d>().swap(lidar_points);

    all_lidar_points.clear();
    std::vector<std::vector<Eigen::Vector3d>>().swap(all_lidar_points);

    all_normals.clear();
    std::vector<Eigen::Vector3d>().swap(all_normals);

    images.clear();
    std::map<std::string, cv::Mat>().swap(images);

    pcds.clear();
    std::map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr>().swap(pcds);

    std::cout << "terminate node" << std::endl;
  }
};

}  // namespace extrinsic

// MEMO : interface , check bug, error, param and aggregation , customize functions
class FacadeCalib {
 private:
  cv::Mat intrinsic_param, dist_coeffs;
  cv::Mat rotation_matrix;
  cv::Mat translation_matrix;
  // TODO : condition check
  std::unique_ptr<intrinsic::MonoCamCalib> mono_camera;
  std::unique_ptr<extrinsic::CameraLidarCalib> camera_lidar;
  std::unique_ptr<intrinsic::StereoCamCalib> stereo_camera;

  int32_t mono_camera_width, mono_camera_height;

 public:
  explicit FacadeCalib() : mono_camera_width(0), mono_camera_height(0) {
    mono_camera = std::make_unique<intrinsic::MonoCamCalib>();
    camera_lidar = std::make_unique<extrinsic::CameraLidarCalib>();
  }

  FacadeCalib(const FacadeCalib &obj) = delete;
  FacadeCalib(const FacadeCalib &&obj) = delete;
  virtual ~FacadeCalib() = default;
  void get_intrinsic_camera_info() {
  }
  bool intrinsicGetImageResolution(const std::string &&file_name, std::vector<int32_t> &res) noexcept {
    if (!mono_camera->getImageResolution(std::move(file_name), res)) {
      return false;
    }
    mono_camera_width = res[0];
    mono_camera_height = res[1];
    return true;
  }
  bool intrinsicGetDecodedImage(const std::string &&file_name, std::vector<uchar> &decoded_image) noexcept {
    if (mono_camera_height != 0 && mono_camera_width != 0) {
      decoded_image.reserve(static_cast<int32_t>(mono_camera_height * mono_camera_width));
    }
    if (!mono_camera->getDecodedImage(std::move(file_name), decoded_image)) {
      return false;
    }
    return true;
  }
  bool intrinsicReadImage(const std::string &&file_name) noexcept {
    if (!mono_camera->readImage(std::move(file_name))) {
      return false;
    }
    return true;
  }
  bool intrinsicClear() {
    if (!mono_camera->clear()) {
      return false;
    }
    return true;
  }
  bool intrinsicReadCheckerboardParams(const double &&dx, const double &&dy, const int &&rows, const int &&cols) noexcept {
    if (!mono_camera->readCheckerboardParams(std::move(dx), std::move(dy), std::move(rows), std::move(cols))) {
      return false;
    }
    return true;
  }
  bool intrinsicGetCalibResult(const std::string &&file_name, std::vector<double> &R_result, std::vector<double> &T_result, std::vector<double> &error_value) noexcept {
    if (!mono_camera->getCalibResult(std::move(file_name), R_result, T_result, error_value)) {
      return false;
    }
    return true;
  }
  bool intrinsicPreprocess(const std::string &&file_name, std::vector<uchar> &debug_image) noexcept {
    if (mono_camera_height != 0 && mono_camera_width != 0) {
      debug_image.reserve(static_cast<int32_t>(mono_camera_height * mono_camera_width));
    }
    if (!mono_camera->preSolveCalib(std::move(file_name), debug_image)) {
      return false;
    }
    return true;
  }

  bool intrinsicRemoveImage(const std::string &&file_name) noexcept {
    if (!mono_camera->removeImage(std::move(file_name))) {
      return false;
    }
    return true;
  }
  bool intrinsicSolveCalib(std::vector<double> &result) noexcept {
    if (!mono_camera->solveCalib(result)) {
      return false;
    }
    return true;
  }

  template <typename T>
  bool cameraLidargetDecodedData(const std::string &&file_name, std::vector<T> &decoded_data) noexcept {
    if (std::is_same<decltype(*decoded_data.begin()), double>::value == true) {
      if (!camera_lidar->getDecodePCD(std::move(file_name), decoded_data)) {
        return false;
      }
    } else if (std::is_same<decltype(*decoded_data.begin()), uchar>::value == true) {
      if (!camera_lidar->getDecodeImage(std::move(file_name), decoded_data)) {
        return false;
      }
    } else {
      return false;
    }
    return true;
  }
  bool cameraLidarPreprocess(const std::string &&file_pcd_name, const std::string &&file_image_name) noexcept {
    // check file order,
    if (!camera_lidar->preSolveCalibPCD(std::move(file_pcd_name))) {
      return false;
    }
    if (!camera_lidar->preSolveCalibImage(std::move(file_image_name))) {
      return false;
    }

    return true;
  }
  bool cameraLidarSolveCalib(std::vector<double> &result) {
    result.reserve(12);
    if (!camera_lidar->solveCalib(result)) {
      return false;
    }
    return true;
  }
  bool cameraLidarGetResultion(const std::string &&file_name, std::vector<int32_t> &res, const std::string &&type) {
    res.reserve(2);
    if (type == "image") {
      if (!camera_lidar->getImageResolution(std::move(file_name), res)) {
        return false;
      }
    } else if (type == "pcd") {
      if (!camera_lidar->getPCDSize(std::move(file_name), res)) {
        return false;
      }
    } else {
      return false;
    }
    return true;
  }
  bool cameraLidarReadData(const std::string &&file_name, const std::string &&type) {
    if (type == "image") {
      if (!camera_lidar->readImage(std::move(file_name))) {
        return false;
      }
    } else if (type == "pcd") {
      if (!camera_lidar->readPCD(std::move(file_name))) {
        return false;
      }
    } else {
      return false;
    }
    return true;
  }
  bool cameraLidarRemoveData(const std::string &&file_name, const std::string &&type) {
    if (type == "image") {
      if (!camera_lidar->removeImage(std::move(file_name))) {
        return false;
      }
    } else if (type == "pcd") {
      if (!camera_lidar->removePCD(std::move(file_name))) {
        return false;
      }
    } else {
      return false;
    }
    return true;
  }
  bool cameraLidarClearData(const std::string &&type) {
    if (type == "image") {
      if (!camera_lidar->clearImage()) {
        return false;
      }
    } else if (type == "pcd") {
      if (!camera_lidar->clearPCD()) {
        return false;
      }
    } else {
      return false;
    }
    return true;
  }
  bool cameraLidarReadCheckerboardParams(const double &&dx, const double &&dy, const int &&rows, const int &&cols) {
    if (!camera_lidar->readCheckerboardParams(std::move(dx), std::move(dy), std::move(rows), std::move(cols))) {
      return false;
    }
    return true;
  }
  bool cameraLidarReadCheckerboardPosition(const std::vector<double> &&filter_info) {
    if (!camera_lidar->readCheckerPosition(std::move(filter_info))) {
      return false;
    }
    return true;
  }

  bool cameraLidarReadCameraInfo(const std::vector<double> &&camera_info) {
    if (!camera_lidar->readCameraParams(std::move(camera_info))) {
      return false;
    }
    return true;
  }
  bool stereoReadImage(const std::string left_name, const std::string right_name) {
    // TODO :exception process only left,right
    std::string sum_name = left_name + ',' + right_name;
    if (!stereo_camera->readImage(std::move(sum_name))) {
      return false;
    }
    return true;
  }
  bool stereoReadCheckerboardParams(double &&dx, double &&dy, int &&rows, int &&cols) {
    if (!stereo_camera->readCheckerboardParams(std::move(dx), std::move(dy), std::move(rows), std::move(cols))) {
      return false;
    }
    return true;
  }
  bool stereoPreprocess(const std::string &&left_name, const std::string &&right_name, std::vector<uchar> &debug) {
    std::string sum_name = left_name + ',' + right_name;
    // TODO : debug reserve
    if (!stereo_camera->preSolveCalib(std::move(sum_name), debug)) {
      return false;
    }
    return true;
  }
  bool stereoSolveCalib(std::vector<double> &result) {
    if (!stereo_camera->solveCalib(result)) {
      return false;
    }
    return true;
  }

  bool get_intrinsic_param_info() {
    cv::Mat intrinsic_param, dist_coeffs;
    cv::Mat rotation_matrix;
    cv::Mat translation_matrix;

    std::shared_ptr<std::vector<cv::Point2f>> image_points;
    std::shared_ptr<std::vector<cv::Point3f>> object_points;
    // std::shared_ptr<int32_t[]> resolution;
    std::vector<int32_t> resolution;
    resolution.reserve(2);
    camera_lidar->getParam(resolution, image_points, object_points);
    cv::calibrateCamera(*object_points, *image_points, cv::Size(resolution[0], resolution[1]), intrinsic_param, dist_coeffs, rotation_matrix, translation_matrix);
    std::vector<double> camera_info;
    camera_info.reserve(11);
    camera_info[0] = resolution[0];
    camera_info[1] = resolution[1];
    for (int i = 0; i < static_cast<int32_t>(dist_coeffs.rows); i++) {
      for (int j = 0; j < static_cast<int32_t>(dist_coeffs.cols); j++) {
        camera_info[2 + (i + 1) * (j)] = dist_coeffs.at<double>(i, j);
      }
    }
    camera_info[7] = intrinsic_param.at<double>(0, 0);
    camera_info[8] = intrinsic_param.at<double>(1, 1);
    camera_info[9] = intrinsic_param.at<double>(0, 2);
    camera_info[10] = intrinsic_param.at<double>(1, 2);
    if (!camera_lidar->readCameraParams(std::move(camera_info))) {
      return false;
    }
    return true;
  }
};