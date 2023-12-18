#include "calibration.h"
// extrinsic::CameraLidarCalib Extrinsic_Calib;

bool extrinsic::CameraLidarCalib::readCameraParams(const std::vector<double>&& camera_info) noexcept {
  image_height = std::move(static_cast<int>(camera_info[0]));
  image_width = std::move(static_cast<int>(camera_info[1]));
  dist_coeffs.at<double>(0) = std::move(camera_info[2]);
  dist_coeffs.at<double>(1) = std::move(camera_info[3]);
  dist_coeffs.at<double>(2) = std::move(camera_info[4]);
  dist_coeffs.at<double>(3) = std::move(camera_info[5]);
  dist_coeffs.at<double>(4) = std::move(camera_info[6]);
  projection_matrix.at<double>(0, 0) = std::move(camera_info[7]);
  projection_matrix.at<double>(1, 1) = std::move(camera_info[8]);
  projection_matrix.at<double>(0, 2) = std::move(camera_info[9]);
  projection_matrix.at<double>(1, 2) = std::move(camera_info[10]);
  projection_matrix.at<double>(2, 2) = std::move(1);
  camera_param_set = 0x1;
  std::cout << "====camera info====" << std::endl;
  std::cout << "intinsic : " << std::endl
            << projection_matrix << std::endl;
  std::cout << "dist_coeffs : " << std::endl
            << dist_coeffs << std::endl;
  std::cout << "image_width : " << std::endl
            << image_width << std::endl;
  std::cout << "image_height : " << std::endl
            << image_height << std::endl;
  return true;
}

bool extrinsic::CameraLidarCalib::readCheckerboardParams(const double&& dx, const double&& dy, const int&& rows, const int&& cols) noexcept {
  checkerboard_rows = std::move(rows);
  checkerboard_cols = std::move(cols);
  checkerboard_dx = std::move(dx);
  checkerboard_dy = std::move(dy);
  for (int i = 0; i < checkerboard_rows; i++)
    for (int j = 0; j < checkerboard_cols; j++)
      object_points.emplace_back(cv::Point3f(j * checkerboard_dy, i * checkerboard_dx, 0.0));
  checkerboard_param_set = 0x10;
  std::cout << "====checkboard info====" << std::endl;
  std::cout << checkerboard_dx << std::endl
            << checkerboard_dy << std::endl
            << checkerboard_rows << std::endl
            << checkerboard_cols << std::endl;
  return true;
}

bool extrinsic::CameraLidarCalib::readCheckerPosition(const std::vector<double>&& filter_info) noexcept {
  x_min = std::make_unique<double>(filter_info[0]);
  x_max = std::make_unique<double>(filter_info[1]);
  y_min = std::make_unique<double>(filter_info[2]);
  y_max = std::make_unique<double>(filter_info[3]);
  z_min = std::make_unique<double>(filter_info[4]);
  z_max = std::make_unique<double>(filter_info[5]);
  filter_param_set = 0x100;
  std::cout << "==== filter info ====" << std::endl;
  std::cout << x_min << std::endl
            << x_max << std::endl
            << y_min << std::endl
            << y_max << std::endl
            << z_min << std::endl
            << z_max << std::endl;
  return true;
}

bool extrinsic::CameraLidarCalib::imageHandler(const cv::Mat& image) {
  if (!checkSumToReady()) {
    std::cout << "fill parameter!" << std::endl;  // TODO : classify what type of problem
  }
  image_points.clear();
  std::vector<cv::Point2f>().swap(image_points);
  projected_points.clear();
  std::vector<cv::Point2f>().swap(projected_points);

  try {
    if (false == cv::findChessboardCorners(image, cv::Size(checkerboard_cols, checkerboard_rows), image_points, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE)) {
      std::cout << "can not find corner !!" << std::endl;
      return false;
    }

    cv::solvePnP(object_points, image, projection_matrix, dist_coeffs, cv_rotation_matrix, cv_translation_vector, false);
    cv::projectPoints(object_points, cv_rotation_matrix, cv_translation_vector, projection_matrix, dist_coeffs, projected_points, cv::noArray());
    cv::Rodrigues(cv_rotation_matrix, cv_c_r_w);
    cv::cv2eigen(cv_c_r_w, c_R_w);

    c_t_w = Eigen::Vector3d(cv_translation_vector.at<double>(0), cv_translation_vector.at<double>(1), cv_translation_vector.at<double>(2));
    r3 = c_R_w.block<3, 1>(0, 2);
    Nc = (r3.dot(c_t_w)) * r3;
    all_normals.push_back(Nc);

  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}

bool extrinsic::CameraLidarCalib::cloudHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud) {
  if (!checkSumToReady()) {
    std::cout << "fill parameter!" << std::endl;  // TODO : classify what type of problem
  }
  std::cout << in_cloud->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  /// Pass through filters
  pcl::PassThrough<pcl::PointXYZI> pass_x;
  pass_x.setInputCloud(in_cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(*x_min, *x_max);
  pass_x.filter(*cloud_filtered_x);

  pcl::PassThrough<pcl::PointXYZI> pass_y;
  pass_y.setInputCloud(cloud_filtered_x);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(*y_min, *y_max);
  pass_y.filter(*cloud_filtered_y);

  pcl::PassThrough<pcl::PointXYZI> pass_z;
  pass_z.setInputCloud(cloud_filtered_y);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(*z_min, *z_max);
  pass_z.filter(*cloud_filtered_z);
  /// Plane Segmentation
  pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(cloud_filtered_z));
  pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
  ransac.setDistanceThreshold(ransac_threshold);
  ransac.computeModel();
  std::vector<int> inliers_indicies;
  ransac.getInliers(inliers_indicies);
  pcl::copyPointCloud<pcl::PointXYZI>(*cloud_filtered_z, inliers_indicies, *plane);

  /// Statistical Outlier Removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(plane);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1);
  sor.filter(*plane_filtered);

  /// Store the points lying in the filtered plane in a vector
  lidar_points.clear();
  std::vector<Eigen::Vector3d>().swap(lidar_points);

  for (size_t i = 0; i < plane_filtered->points.size(); i++) {
    double X = plane_filtered->points[i].x;
    double Y = plane_filtered->points[i].y;
    double Z = plane_filtered->points[i].z;
    lidar_points.push_back(Eigen::Vector3d(X, Y, Z));
  }

  // ROS_INFO_STREAM("No of planar_pts: " << lidar_points.size());
  std::cout << "No of planar_pts: " << plane_filtered->points.size() << std::endl;
  if (plane_filtered->points.size() < min_points_on_plane) return false;
  all_lidar_points.push_back(lidar_points);

  return true;
}

bool extrinsic::CameraLidarCalib::solveWithPCL() {
  try {
    rotation_matrix << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    Eigen::Vector3d axis_angle;
    ceres::RotationMatrixToAngleAxis(rotation_matrix.data(), axis_angle.data());  //  convert to Rodrigus type

    Eigen::Vector6d opt_param;

    opt_param(0) = axis_angle(0);
    opt_param(1) = axis_angle(1);
    opt_param(2) = axis_angle(2);
    opt_param(3) = translation_vector(0);
    opt_param(4) = translation_vector(1);
    opt_param(5) = translation_vector(2);

    ceres::LossFunction* loss_function = NULL;
    ceres::Problem problem;
    problem.AddParameterBlock(opt_param.data(), opt_param.size());

    for (int i = 0; i < static_cast<int>(all_normals.size()); i++) {
      Eigen::Vector3d normal_i = all_normals[i];
      std::vector<Eigen::Vector3d> lidar_point_i = all_lidar_points[i];
      for (int j = 0; j < lidar_point_i.size(); j++) {
        Eigen::Vector3d lidar_point = lidar_point_i[j];
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<extrinsic::TargetBasedErrorTerm, 1, 6>(new extrinsic::TargetBasedErrorTerm(lidar_point, normal_i));
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
    ceres::AngleAxisToRotationMatrix(axis_angle.data(), rotation_matrix.data());
    translation_vector(0) = opt_param(3);
    translation_vector(1) = opt_param(4);
    translation_vector(2) = opt_param(5);
    std::cout << "====result====" << std::endl;
    std::cout << "rot" << std::endl
              << rotation_matrix << std::endl;
    std::cout << "trans" << std::endl
              << translation_vector << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }
}

/**
 * @brief return decoded pcd data
 *
 * @param file_name : pcd file name for decode
 * @param decoded_data : return pcd value
 */
template <typename T>
bool extrinsic::CameraLidarCalib::getDecodePCD(const std::string&& file_name, std::vector<T>& decoded_data) noexcept {
  auto index = pcds.find(file_name);
  if (index == pcds.end()) {
    std::cout << "remove : can not find the file name" << std::endl;
    return false;
  }
  for (int i = 0; i < index->second->points.size(); i++) {
    decoded_data[i * channel] = index->second->points[i].x;
    decoded_data[i * channel + 1] = index->second->points[i].y;
    decoded_data[i * channel + 2] = index->second->points[i].z;
    decoded_data[i * channel + 3] = index->second->points[i].intensity;
  }

  return true;
}

/**
 * @brief return the Decoded Image(**png only**)
 *
 * @param file_name : image file name
 * @param decoded_data : return image value
 */
template <typename T>
bool extrinsic::CameraLidarCalib::getDecodeImage(const std::string&& file_name, std::vector<T>& decoded_data) noexcept {
  auto index = images.find(file_name);
  if (index == images.end()) {
    std::cout << "remove : can not find the file name" << std::endl;
    return false;
  }
  for (int row = 0; row < index->second.rows; row++) {
    for (int col = 0; col < index->second.cols; col++) {
      decoded_data[row * col] = index->second.at<uchar>(row, col);
    }
  }
  return true;
}

bool extrinsic::CameraLidarCalib::preSolveCalibPCD(const std::string&& file_name) noexcept {
  auto index = pcds.find(file_name);
  if (index == pcds.end()) {
    std::cout << "can not find the file name" << std::endl;
    return false;
  }
  if (!cloudHandler(index->second)) {
    return false;
  }
  return true;
}
bool extrinsic::CameraLidarCalib::preSolveCalibImage(const std::string&& file_name) noexcept {
  auto index = images.find(file_name);
  if (index == images.end()) {
    std::cout << "can not find the file name" << std::endl;
    return false;
  }
  if (!imageHandler(index->second)) {
    return false;
  }
  return true;
}

/**
 * @brief return extrinsic results
 *
 * @param result : rotation matrix [0,0] -> [0,1] ->[0,2] ->[1,0] .. translation[0] ->translation[1]..
 * @return int
 */
bool extrinsic::CameraLidarCalib::solveCalib(std::vector<double>& result) noexcept {
  if (!solveWithPCL()) {
    return false;
  }
  std::cout << "fisnish calib" << std::endl;
  for (int row = 0; row < rotation_matrix.rows(); row++) {
    for (int col = 0; col < rotation_matrix.cols(); col++) {
      result[(row + 1) * col] = rotation_matrix(row, col);
    }
  }
  result[9] = translation_vector(0);
  result[10] = translation_vector(1);
  result[11] = translation_vector(2);

  return true;
}

/**
 * @brief Return the Image Resolution
 *
 * @param file_name : query image name
 * @param width : return value
 * @param height : return value,,size, channel
 */

bool extrinsic::CameraLidarCalib::getImageResolution(const std::string file_name, std::vector<int32_t>& res) {
  auto index = images.find(file_name);
  if (index == images.end()) {
    std::cout << "can not find the file name" << std::endl;
    return false;
  }
  res[0] = index->second.cols;
  res[1] = index->second.rows;
  return true;
}

bool extrinsic::CameraLidarCalib::getPCDSize(const std::string file_name, std::vector<int32_t>& res) {
  auto index = pcds.find(file_name);
  if (index == pcds.end()) {
    std::cout << "can not find the file name" << std::endl;
    return false;
  }
  res[0] = index->second->width * index->second->height;
  res[1] = channel;  // TODO : make channel dynamic
  return true;
}

bool extrinsic::CameraLidarCalib::readImage(const std::string&& file_name) {
  try {
    cv::Mat img = cv::imread(file_name);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    images.insert({file_name, gray});
    std::cout << "res : " << gray.rows << "," << gray.cols << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}
bool extrinsic::CameraLidarCalib::readPCD(const std::string&& file_name) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  try {
    pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *cloud);
    if (cloud->width * cloud->height == 0) {
      std::cout << "empty in pcd file" << std::endl;
      return false;
    }
    pcds.insert({file_name, cloud});
    std::cout << cloud->width << "," << cloud->height << std::endl;

  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}

/**
 * @brief remove the query image in map
 */
bool extrinsic::CameraLidarCalib::removeImage(const std::string&& file_name) {
  auto index = images.find(file_name);
  if (index == images.end()) {
    std::cout << "can not find the file name" << std::endl;
    return false;
  }
  images.erase(index);
  return true;
}
bool extrinsic::CameraLidarCalib::removePCD(const std::string&& file_name) {
  auto index = pcds.find(file_name);
  if (index == pcds.end()) {
    std::cout << "can not find the file name" << std::endl;
    return false;
  }
  pcds.erase(index);
  return true;
}

bool extrinsic::CameraLidarCalib::clearImage() {
  images.clear();
  std::map<std::string, cv::Mat>().swap(images);
  return 1;
}

bool extrinsic::CameraLidarCalib::clearPCD() {
  pcds.clear();
  std::map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr>().swap(pcds);
  return 1;
}

// extern "C" {

// int EMSCRIPTEN_KEEPALIVE readCheckerboardParams(double dx, double dy, int rows, int cols) {
//   if (1 == (Extrinsic_Calib.readCheckerboardParams(dx, dy, rows, cols))) {
//     return 1;
//   } else {
//     return -1;
//   }
// }

// int  readCameraParams(std::shared_ptr<double[]> camera_info) {
//   if (1 == (Extrinsic_Calib.readCameraParams(std::move(camera_info)))) {
//     return 1;
//   } else {
//     return -1;
//   }

//   return 1;
// }

// int EMSCRIPTEN_KEEPALIVE readCheckerPosition(std::shared_ptr<double[]> filter_info) {
//   if (1 == (Extrinsic_Calib.readCheckerPosition(std::move(filter_info)))) {
//     return 1;
//   } else {
//     return -1;
//   }

//   return 1;
// }
// }
