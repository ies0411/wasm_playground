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

#include <emscripten.h>
#include <emscripten/bind.h>
#include <opencv2/calib3d/calib3d_c.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

// TODO : roslaunch file and set param
class setCalibEnv {
   private:
    // var

    int checkerboard_rows_num_, checkerboard_colm_num_;
    std::string load_path_;  // MEMO, result file type : 1. json, 2.txt
    double dx_, dy_;         // MEMO . unit : mm
    std::vector<std::vector<cv::Point3f>> objpoints_;
    std::vector<std::vector<cv::Point2f>> imgpoints_;
    std::vector<cv::Point3f> object_;

    int view_cnt_ = 0;

   public:
    int width_, height_;
    int img_size_ = 0;
    cv::Mat cameraMatrix_, distCoeffs_;
    std::vector<std::string> file_order_;
    // int size = calib.images_.size();
    cv::Mat R_;
    //(size, 3, CV_64F);
    cv::Mat T_;
    //(size, 3, CV_64F);
    // std::vector<cv::Mat> images_;
    // std::vector<std::pair<cv::Mat, std::string>> images_;

    std::map<std::string, cv::Mat> images_;
    // m;
    setCalibEnv() {
        std::cout << "generate intrinsic class" << std::endl;
    }

    std::string getImagefilepath() {
        return load_path_;
    }

    int calibRawimage(cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &R, cv::Mat &T);
    bool preSolveCalib(std::string file_name, cv::Mat &frame);

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

    ~setCalibEnv();
};

setCalibEnv::~setCalibEnv() {
    // TODO : check memory leak
    std::cout << "terminate intrinsic class" << std::endl;
}

/**
 * @brief callback function of Image raw data
 *
 * @param msg : images data with ros msgs type
 */

bool setCalibEnv::preSolveCalib(std::string file_name, cv::Mat &frame) {
    std::cout << objpoints_.size() << std::endl;
    std::vector<cv::Point2f> corner_pts;
    // cv::Mat frame;
    // cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    // std::cout << frame.size().width << "." << frame.size().height << std::endl;
    bool success = cv::findChessboardCorners(frame, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (success) {
        std::cout << "success" << std::endl;
        cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
        cv::cornerSubPix(frame, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        objpoints_.push_back(object_);
        imgpoints_.push_back(corner_pts);
        file_order_.push_back(file_name);
        cv::drawChessboardCorners(frame, cv::Size(checkerboard_colm_num_, checkerboard_rows_num_), corner_pts, success);

        return true;

    } else {
        std::cout << "CAN NOT FIND BOARD CORNER" << std::endl;
        return false;
        // img_size_--;
        // if (view_cnt_ < img_size_) ;
    }
}

int setCalibEnv::calibRawimage(cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &R, cv::Mat &T) {
    std::cout << "Calib.." << std::endl;
    // cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::calibrateCamera(objpoints_, imgpoints_, cv::Size(images_.begin()->second.cols, images_.begin()->second.rows), cameraMatrix, distCoeffs, R, T);

    std::cout << "R : "
              << R << std::endl;
    std::cout << R.size() << std::endl;
    std::cout << "T : "
              << T << std::endl;

    std::cout << "cameraMatrix : \n"
              << cameraMatrix << std::endl;
    std::cout << cameraMatrix.size() << std::endl;
    std::cout << "distCoeffs : \n"
              << distCoeffs << std::endl;

    // TODO :exception process
    return 1;
}
