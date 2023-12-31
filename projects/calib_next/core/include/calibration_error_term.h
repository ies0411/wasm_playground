#pragma once

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace extrinsic {

class TargetBasedErrorTerm {
 private:
  const Eigen::Vector3d laser_point_;
  const Eigen::Vector3d normal_to_plane_;

 public:
  TargetBasedErrorTerm(const Eigen::Vector3d& laser_point,
                       const Eigen::Vector3d& normal_to_plane) : laser_point_(laser_point), normal_to_plane_(normal_to_plane) {}

  template <typename T>
  bool operator()(const T* const R_t, T* residual) const {
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
}  // namespace extrinsic
