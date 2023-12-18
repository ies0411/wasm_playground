

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/rotation.h>
#include <emscripten.h>
#include <emscripten/bind.h>
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
#include <pcl/segmentation/sac_segmentation.h>
#include <string.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
extern "C" {

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

int EMSCRIPTEN_KEEPALIVE readPCD(const char *file_name, double *points_size) {
  const std::string file_name_cpp(file_name);
  // std::cout << file_name_cpp << std::endl;
  try {
    pcl::io::loadPCDFile<pcl::PointXYZI>(file_name_cpp, *cloud);
    if (cloud->width * cloud->height == 0) {
      std::cout << "empty in pcd file" << std::endl;
      return -1;
    }
    // std::cout << "width, height : " << cloud->width << "," << cloud->height << std::endl;
    points_size[0] = cloud->width;
    points_size[1] = cloud->height;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return -1;
  }
  return 1;
}
}
// MEMO : Dont use modern pointer
extern "C" {
int EMSCRIPTEN_KEEPALIVE decodePCD(double *decoded_pcd) {
  for (int i = 0; i < cloud->points.size(); i++) {
    decoded_pcd[i * 4] = cloud->points[i].x;
    decoded_pcd[i * 4 + 1] = cloud->points[i].y;
    decoded_pcd[i * 4 + 2] = cloud->points[i].z;
    decoded_pcd[i * 4 + 3] = cloud->points[i].intensity;
  }
  return 1;
}
}

extern "C" {
int EMSCRIPTEN_KEEPALIVE findPlane(double *coeff) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);      //(옵션) // Enable model coefficient refinement (optional).
  seg.setInputCloud(cloud);               //입력
  seg.setModelType(pcl::SACMODEL_PLANE);  //적용 모델  // Configure the object to look for a plane.
  seg.setMethodType(pcl::SAC_RANSAC);     //적용 방법   // Use RANSAC method.
  seg.setMaxIterations(1000);             //최대 실행 수
  seg.setDistanceThreshold(0.01);         // inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
  // seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
  seg.segment(*inliers, *coefficients);  //세그멘테이션 적용

  std::cout << "Model coefficients: " << std::endl
            << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;

  coeff[0] = coefficients->values[0];
  coeff[1] = coefficients->values[1];
  coeff[2] = coefficients->values[2];
  coeff[3] = coefficients->values[3];

  return 1;
}
}
