#include "calibration.h"

bool intrinsic::MonoCamCalib::preSolveCalib(const std::string &&file_name, std::vector<uchar> &debug_image) {
  auto index = images.find(file_name);
  // std::cout << index->first << std::endl;
  if (index == images.end()) {
    std::cout << "presolve : can not find the file name" << std::endl;
    return false;
  }
  std::vector<cv::Point2f> corner_pts;
  bool success;
  try {
    // std::cout << checkerboard_colm_num << "," << checkerboard_rows_num << std::endl;

    success = cv::findChessboardCorners(index->second, cv::Size(checkerboard_colm_num, checkerboard_rows_num), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    std::cout << "error in finding checkerboard";
    return false;
  }

  if (success) {
    // std::cout << "success" << std::endl;
    cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
    cv::cornerSubPix(index->second, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);  // TODO : change waiting option
    object_points.push_back(object);
    image_points.push_back(corner_pts);
    file_order.push_back(file_name);
    cv::drawChessboardCorners(index->second, cv::Size(checkerboard_colm_num, checkerboard_rows_num), corner_pts, success);
    for (int row = 0; row < static_cast<int>(index->second.rows); row++) {
      for (int col = 0; col < static_cast<int>(index->second.cols); col++) {
        debug_image[row * col] = index->second.at<uchar>(row, col);
      }
    }
  } else {
    std::cout << "error presolve calib " << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief Get the Image Resolution object
 *
 * @param file_name : image name
 * @param res : resolution of the image
 * @return int : resolution (width, height)
 */
bool intrinsic::MonoCamCalib::getImageResolution(const std::string &&file_name, std::vector<int32_t> &res) noexcept {
  auto index = images.find(file_name);
  if (index == images.end()) {
    std::cout << "getRes : can not find the file name" << std::endl;
    return false;
  }
  std::cout << index->second.cols << std::endl;

  res[0] = index->second.cols;
  res[1] = index->second.rows;

  return true;
}

/**
 * @brief run calibration
 *
 * @param result
 * result[0] -> result[8]
 * k1 k2 p1 p2 p3 fx fy cx cy
 */
// TODO : check whether using p3 or not
bool intrinsic::MonoCamCalib::solveCalib(std::vector<double> &result) {
  try {
    cv::calibrateCamera(object_points, image_points, cv::Size(images.begin()->second.cols, images.begin()->second.rows), intrinsic_param, dist_coeffs, rotation_matrix, translation_matrix);
    std::cout << "====result====" << std::endl;
    std::cout << "rotation_matrix : "
              << rotation_matrix << std::endl;
    std::cout << rotation_matrix.size() << std::endl;
    std::cout << "translation_matrix : "
              << translation_matrix << std::endl;
    std::cout << "intrinsic_param : \n"
              << intrinsic_param << std::endl;
    std::cout << intrinsic_param.size() << std::endl;
    std::cout << "dist_coeffs : \n"
              << dist_coeffs << std::endl;
  } catch (const std::exception &e) {
    std::cout << "error in calib function" << std::endl;
    std::cerr << e.what() << '\n';
    return false;
  }
  for (int i = 0; i < static_cast<int32_t>(dist_coeffs.rows); i++) {
    for (int j = 0; j < static_cast<int32_t>(dist_coeffs.cols); j++) {
      result[(i + 1) * (j)] = dist_coeffs.at<float>(i, j);
    }
  }
  result[5] = intrinsic_param.at<float>(0, 0);
  result[6] = intrinsic_param.at<float>(1, 1);
  result[7] = intrinsic_param.at<float>(0, 2);
  result[8] = intrinsic_param.at<float>(1, 2);
  return true;
}

/**
 * @brief register checkerboard info
 *
 * @param dx : col size
 * @param dy : row size
 * @param rows : row number
 * @param cols : col number
 */
bool intrinsic::MonoCamCalib::readCheckerboardParams(const double &&dx, const double &&dy, const int &&rows, const int &&cols) {
  try {
    checkerboard_rows_num = std::move(rows);
    checkerboard_colm_num = std::move(cols);
    checkerboard_dx = std::move(dx);
    checkerboard_dy = std::move(dy);
    for (int i = 0; i < checkerboard_rows_num; i++) {
      for (int j = 0; j < checkerboard_colm_num; j++) {
        object.emplace_back(cv::Point3f(j * checkerboard_dy, i * checkerboard_dx, 0.0));
      }
    }
    std::cout << "====checkerboard info====" << std::endl;
    std::cout << checkerboard_dx << std::endl
              << checkerboard_dy << std::endl
              << checkerboard_rows_num << std::endl
              << checkerboard_colm_num << std::endl;

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}

/**
 * @brief Get the Decoded Image object
 *
 * @param file_name Image name
 * @param decoded_image file for decoding
 * @return 1 : success, -1 : can not find the file name in map
 */
bool intrinsic::MonoCamCalib::getDecodedImage(const std::string &&file_name, std::vector<uchar> &decoded_image) {
  auto index = images.find(file_name);
  if (index == images.end()) {
    std::cout << "remove : can not find the file name" << std::endl;
    return false;
  }
  for (int row = 0; row < static_cast<int>(index->second.rows); row++) {
    for (int col = 0; col < static_cast<int>(index->second.cols); col++) {
      decoded_image[row * col] = index->second.at<uchar>(row, col);
    }
  }
  return true;
}

/**
 * @brief remove Image from map
 *
 * @param file_name : image file name
 * @return 1 : success, -1 : can not find the file name in map
 */
bool intrinsic::MonoCamCalib::removeImage(const std::string &&file_name) {
  auto index = images.find(file_name);
  if (index == images.end()) {
    std::cout << "remove : can not find the file name" << std::endl;
    return false;
  }
  images.erase(index);
  return true;
}

/**
 * @brief read image and insert to map
 * @param file_name : image name
 */
bool intrinsic::MonoCamCalib::readImage(const std::string &&file_name) {
  try {
    // std::cout << file_name << std::endl;
    cv::Mat img = cv::imread(file_name, cv::IMREAD_UNCHANGED);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

    images.insert({file_name, gray});

    return true;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }
}

/**
 * @brief clear map
 */
bool intrinsic::MonoCamCalib::clear() {
  try {
    images.clear();
    std::map<std::string, cv::Mat>().swap(images);
    return true;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }
}

bool intrinsic::MonoCamCalib::getCalibResult(const std::string &&file_name, std::vector<double> &R_result, std::vector<double> &T_result, std::vector<double> &error_value) noexcept {
  bool file_check = false;
  for (int i = 0; i < file_order.size(); i++) {
    if (file_order[i] == file_name) {
      // TODO:implements
      R_result[0] = rotation_matrix.at<double>(i, 0);
      R_result[1] = rotation_matrix.at<double>(i, 1);
      R_result[2] = rotation_matrix.at<double>(i, 2);
      T_result[0] = translation_matrix.at<double>(i, 0);
      T_result[1] = translation_matrix.at<double>(i, 1);
      T_result[2] = translation_matrix.at<double>(i, 2);
      file_check = true;
    }
  }
  if (!file_check) {
    std::cout << "result : can not find file name" << std::endl;
    return -1;
  }
  return 1;
}
bool intrinsic::StereoCamCalib::parsingFilename(const std::string &&file_names, std::vector<std::string> &file_names_vector) {
  std::stringstream sstream(file_names);
  std::string file_name;
  file_names_vector.reserve(2);
  try {
    while (getline(sstream, file_name, ',')) {
      file_names_vector.push_back(file_name);
    }
    if (file_names_vector.size() > 2) {
      return false;
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }

  return true;
}
bool intrinsic::StereoCamCalib::readImage(const std::string &&file_name) {
  std::vector<std::string> file_names_vector;
  if (!parsingFilename(std::move(file_name), file_names_vector)) {
    return false;
  }
  try {
    uint8_t idx = 0;
    for (auto &p : file_names_vector) {
      cv::Mat img = cv::imread(p, cv::IMREAD_UNCHANGED);
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
      if (idx == static_cast<uint8_t>(StereoCam::LEFT)) {
        images_left.insert({p, gray});
      } else if (idx == static_cast<uint8_t>(StereoCam::RIGHT)) {
        images_right.insert({p, gray});
      } else {
        return false;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}
// TODO : check res, make condition to add to map
bool intrinsic::StereoCamCalib::preSolveCalib(const std::string &&file_name, std::vector<uchar> &debug_image) {
  std::vector<std::string> file_names_vector;
  if (!parsingFilename(std::move(file_name), file_names_vector)) {
    return false;
  }
  std::vector<cv::Point2f> corner_pts_left, corner_pts_right;
  corner_pts_left.reserve(100);
  corner_pts_right.reserve(100);

  uint8_t idx = 0;
  bool success_left, success_right;
  for (auto &p : file_names_vector) {
    if (idx == static_cast<uint8_t>(StereoCam::LEFT)) {
      auto index = images_left.find(p);
      if (index == images_left.end()) {
        std::cout << "presolve : can not find the file name in left images" << std::endl;
        return false;
      }
      bool success_left = cv::findChessboardCorners(index->second, cv::Size(checkerboard_colm_num, checkerboard_rows_num), corner_pts_left, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
      if (!success_left) {
        std::cout << "CAN NOT FIND BOARD CORNER LEFT" << std::endl;
        return false;
      }
      cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
      cv::cornerSubPix(index->second, corner_pts_left, cv::Size(11, 11), cv::Size(-1, -1), criteria);
      cv::drawChessboardCorners(index->second, cv::Size(checkerboard_colm_num, checkerboard_rows_num), corner_pts_left, success_left);
      for (int row = 0; row < static_cast<int>(index->second.rows); row++) {
        for (int col = 0; col < static_cast<int>(index->second.cols); col++) {
          debug_image[row * col] = index->second.at<uchar>(row, col);
        }
      }
    } else if (idx == static_cast<uint8_t>(StereoCam::RIGHT)) {
      auto index = images_right.find(p);
      if (index == images_right.end()) {
        std::cout << "presolve : can not find the file name in left images" << std::endl;
        return false;
      }
      bool success_right = cv::findChessboardCorners(index->second, cv::Size(checkerboard_colm_num, checkerboard_rows_num), corner_pts_right, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
      if (!success_right) {
        std::cout << "CAN NOT FIND BOARD CORNER RIGHT" << std::endl;
        return false;
      }
      cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
      cv::cornerSubPix(index->second, corner_pts_right, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    }
  }

  objpoints_right.push_back(object);
  objpoints_left.push_back(object);
  imgpoints_right.push_back(corner_pts_right);
  imgpoints_left.push_back(corner_pts_left);
}
bool intrinsic::StereoCamCalib::solveCalib(std::vector<double> &result) {
  // TODO : check cv::size(row, col) ? (col,row)?
  cv::Size image_size = images_left.begin()->second.size();

  cv::calibrateCamera(objpoints_left, imgpoints_left, image_size, cameraMatrix_left, distCoeffs_left, R_left, T_left);
  cv::calibrateCamera(objpoints_right, imgpoints_right, image_size, cameraMatrix_right, distCoeffs_right, R_right, T_right);

  cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
                      cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, image_size,
                      rotation_matrix, translation_matrix, essential_matrix, fundamental_matrix, CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
                      cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

  std::cout << "Rotation Matrix\n"
            << rotation_matrix << "\n\n";
  std::cout << "Translation Vector\n"
            << translation_matrix << "\n\n";
  std::cout << "Essential Matrix\n"
            << essential_matrix << "\n\n";
  std::cout << "Fundamental Matrix\n"
            << fundamental_matrix << "\n\n\n";
  // TODO : substitute result
  //  cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, R1, R2, P1, P2, Q);

  // cv::Mat imgU1, imgU2, lmapx, lmapy, rmapx, rmapy;
  // cv::initUndistortRectifyMap(cameraMatrix_left_, distCoeffs_left_, R1_, P1_, imgsize, CV_32FC1, lmapx, lmapy);
  // cv::initUndistortRectifyMap(cameraMatrix_right_, distCoeffs_right_, R2_, P2_, imgsize, CV_32FC1, rmapx, rmapy);

  // std::cout << R1_ << std::endl
  //           << P1_ << std::endl;
  // std::cout << R2_ << std::endl
  //           << P2_ << std::endl;

  // cv::remap(left_img, imgU1, lmapx, lmapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
  // cv::remap(right_img, imgU2, rmapx, rmapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

  // cv::imwrite(std::string(save_path_ + "img_left.png"), imgU1);
  // cv::imwrite(std::string(save_path_ + "img_right.png"), imgU2);
}
