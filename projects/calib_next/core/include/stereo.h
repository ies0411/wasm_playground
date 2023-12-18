
class setStereoCalibEnv {
 private:
  // var
  int checkerboard_rows_num_, checkerboard_colm_num_;

  double dx_, dy_;  // MEMO . unit : mm
  int32_t view_cnt_ = 0;
  // int view_num_threshold_ = 0;

  std::vector<std::vector<cv::Point3f>>
      objpoints_;
  std::vector<std::vector<cv::Point3f>> objpoints_right_, objpoints_left_;
  std::vector<std::vector<cv::Point2f>> imgpoints_;
  std::vector<std::vector<cv::Point2f>> imgpoints_right_, imgpoints_left_;
  std::vector<cv::Point3f> object_;

  // function
 public:
  cv::Mat cameraMatrix_left_, cameraMatrix_right_, distCoeffs_left_, distCoeffs_right_, R_left_, R_right_, T_left_, T_right_;
  cv::Mat R1_, R2_, P1_, P2_, Q_;
  cv::Mat R_, T_, E_, F_;

  std::vector<cv::Mat> images_left_, images_right_;

  int preCalibStereoRawimage(cv::Mat &gray_left, cv::Mat &gray_right);

  int readCheckerboardParams(double &dx, double &dy, int &rows, int &cols) {
    checkerboard_rows_num_ = rows;
    checkerboard_colm_num_ = cols;
    dx_ = dx;
    dy_ = dy;
    for (int i = 0; i < checkerboard_rows_num_; i++)
      for (int j = 0; j < checkerboard_colm_num_; j++)
        object_.emplace_back(cv::Point3f(j * dy_, i * dx_, 0.0));
    std::cout << dx_ << std::endl
              << dy_ << std::endl
              << checkerboard_rows_num_ << std::endl
              << checkerboard_colm_num_ << std::endl;
    return 1;
  }
  int CalibStereoRawimage() {
    std::cout << "calib.." << std::endl;

    cv::calibrateCamera(objpoints_left_, imgpoints_left_, cv::Size(images_left_[0].rows, images_left_[0].cols), cameraMatrix_left_, distCoeffs_left_, R_left_, T_left_);
    cv::calibrateCamera(objpoints_right_, imgpoints_right_, cv::Size(images_right_[0].rows, images_right_[0].cols), cameraMatrix_right_, distCoeffs_right_, R_right_, T_right_);

    cv::Size imgsize = images_right_[0].size();
    cv::stereoCalibrate(objpoints_left_, imgpoints_left_, imgpoints_right_,
                        cameraMatrix_left_, distCoeffs_left_, cameraMatrix_right_, distCoeffs_right_, imgsize,
                        R_, T_, E_, F_, CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
                        cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

    // cv::Rect validRoi[2];
    std::cout << "Rotation Matrix\n"
              << R_ << "\n\n";
    std::cout << "Translation Vector\n"
              << T_ << "\n\n";
    std::cout << "Essential Matrix\n"
              << E_ << "\n\n";
    std::cout << "Fundamental Matrix\n"
              << F_ << "\n\n\n";
    cv::stereoRectify(cameraMatrix_left_, distCoeffs_left_, cameraMatrix_right_, distCoeffs_right_, imgsize,
                      R_, T_, R1_, R2_, P1_, P2_, Q_);

    cv::Mat imgU1, imgU2, lmapx, lmapy, rmapx, rmapy;
    cv::initUndistortRectifyMap(cameraMatrix_left_, distCoeffs_left_, R1_, P1_, imgsize, CV_32FC1, lmapx, lmapy);
    cv::initUndistortRectifyMap(cameraMatrix_right_, distCoeffs_right_, R2_, P2_, imgsize, CV_32FC1, rmapx, rmapy);

    std::cout << R1_ << std::endl
              << P1_ << std::endl;
    std::cout << R2_ << std::endl
              << P2_ << std::endl;

    // cv::remap(left_img, imgU1, lmapx, lmapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    // cv::remap(right_img, imgU2, rmapx, rmapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    // cv::imwrite(std::string(save_path_ + "img_left.png"), imgU1);
    // cv::imwrite(std::string(save_path_ + "img_right.png"), imgU2);
  }
  ~setStereoCalibEnv();
};

setStereoCalibEnv::~setStereoCalibEnv() {
  std::cout << "terminate calibration node" << std::endl;
}

int setStereoCalibEnv::preCalibStereoRawimage(cv::Mat &gray_left, cv::Mat &gray_right) {
  std::vector<cv::Point2f> corner_pts_left, corner_pts_right, sqr_corner_pts_left, sqr_corner_pts_right;
  // cv::Mat gray_left, gray_right;

  // cv::cvtColor(left_img, gray_left, cv::COLOR_BGR2GRAY);
  // cv::cvtColor(right_img, gray_right, cv::COLOR_BGR2GRAY);

  bool success_left = cv::findChessboardCorners(gray_left, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_left, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
  bool success_right = cv::findChessboardCorners(gray_right, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_right, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

  if (!success_left || !success_right) {
    std::cout << "CAN NOT FIND BOARD CORNER" << std::endl;
    return -1;
  }
  cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
  cv::cornerSubPix(gray_left, corner_pts_left, cv::Size(11, 11), cv::Size(-1, -1), criteria);
  // cv::drawChessboardCorners(gray_left, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_left, success_left);
  // cv::imshow("left_image", frame_left);
  cv::cornerSubPix(gray_right, corner_pts_right, cv::Size(11, 11), cv::Size(-1, -1), criteria);
  // cv::drawChessboardCorners(gray_right, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts_right, success_right);
  // cv::imshow("right_image", frame_right);

  // cv::waitKey(10);

  std::cout << "DETECT" << view_cnt_ << std::endl;
  objpoints_right_.push_back(object_);
  objpoints_left_.push_back(object_);
  imgpoints_right_.push_back(corner_pts_right);
  imgpoints_left_.push_back(corner_pts_left);
  return 1;
}