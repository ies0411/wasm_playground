#include "cam_intrinsic.h"

setCalibEnv calib;
extern "C" {

int EMSCRIPTEN_KEEPALIVE getDecodedImage(char* file_name, uchar* decodedImage) {
    std::string file_name_cpp(file_name);
    auto index = calib.images_.find(file_name_cpp);
    if (index == calib.images_.end()) {
        std::cout << "remove : can not find the file name" << std::endl;
        return -1;
    }
    for (int row = 0; row < index->second.rows; row++) {
        for (int col = 0; col < index->second.cols; col++) {
            decodedImage[row * col] = index->second.at<uchar>(row, col);
        }
    }
    return 1;
}

int EMSCRIPTEN_KEEPALIVE removeImage(char* file_name) {
    std::string file_name_cpp(file_name);
    auto index = calib.images_.find(file_name_cpp);
    if (index == calib.images_.end()) {
        std::cout << "remove : can not find the file name" << std::endl;
        return -1;
    }
    calib.images_.erase(index);
    return 1;
}
int EMSCRIPTEN_KEEPALIVE preSolveCalib(char* file_name) {
    std::string file_name_cpp(file_name);
    auto index = calib.images_.find(file_name_cpp);
    std::cout << index->first << std::endl;
    if (index == calib.images_.end()) {
        std::cout << "presolve : can not find the file name" << std::endl;
        return -1;
    }
    if (calib.preSolveCalib(index->first, index->second)) {
        return 1;
    } else {
        // std::string debug_file_name = "/debug" + std::to_string(index) + ".jpg";
        // cv::Mat image = calib.images_[cnt];
        // cv::imwrite(debug_file_name, image);
        return -1;
    }
    // TODO : return debugging img
}

// int getDecodedImage(char* file_name, uint8* decodedImage)
// TODO: decodeimage function

int EMSCRIPTEN_KEEPALIVE getImageResolution(char* file_name, int* res) {
    std::string file_name_cpp(file_name);
    auto index = calib.images_.find(file_name_cpp);
    if (index == calib.images_.end()) {
        std::cout << "getRes : can not find the file name" << std::endl;
        return -1;
    }
    res[0] = index->second.cols;
    res[1] = index->second.rows;

    return 1;
}

int EMSCRIPTEN_KEEPALIVE getCalibResult(char* file_name, double* R_result, double* T_result, double* error_value) {
    std::string file_name_cpp(file_name);
    bool file_check = false;
    for (int i = 0; i < calib.file_order_.size(); i++) {
        if (calib.file_order_[i] == file_name) {
            R_result[0] = calib.R_.at<double>(i, 0);
            R_result[1] = calib.R_.at<double>(i, 1);
            R_result[2] = calib.R_.at<double>(i, 2);
            T_result[0] = calib.T_.at<double>(i, 0);
            T_result[1] = calib.T_.at<double>(i, 1);
            T_result[2] = calib.T_.at<double>(i, 2);
        }
    }
    if (!file_check) {
        std::cout << "result : can not find file name" << std::endl;
        return -1;
    }
    return 1;
}

int EMSCRIPTEN_KEEPALIVE solveCalib(double* result) {
    std::cout << "calib start" << std::endl;
    calib.calibRawimage(calib.cameraMatrix_, calib.distCoeffs_, calib.R_, calib.T_);
    std::cout << "finish_calib" << std::endl;
    for (int i = 0; i < calib.distCoeffs_.rows; i++) {
        for (int j = 0; j < calib.distCoeffs_.cols; j++) {
            result[(i + 1) * (j)] = calib.distCoeffs_.at<double>(i, j);
        }
    }
    result[5] = calib.cameraMatrix_.at<double>(0, 0);
    result[6] = calib.cameraMatrix_.at<double>(1, 1);
    result[7] = calib.cameraMatrix_.at<double>(0, 2);
    result[8] = calib.cameraMatrix_.at<double>(1, 2);

    return 1;
}
int EMSCRIPTEN_KEEPALIVE readImage(char* file_name) {
    std::string file_name_cpp(file_name);
    cv::Mat img = cv::imread(file_name, cv::IMREAD_UNCHANGED);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    // std::string debug_file_name = "/debug.jpg";
    // cv::imwrite(debug_file_name, gray);
    calib.images_.insert({file_name_cpp, gray});
    // std::cout << gray.rows << "," << gray.cols << std::endl;

    return 1;
}
int EMSCRIPTEN_KEEPALIVE clear() {
    calib.images_.clear();
    std::map<std::string, cv::Mat>().swap(calib.images_);
    return 1;
}
int EMSCRIPTEN_KEEPALIVE readCheckerboardParams(double dx, double dy, int rows, int cols) {
    if (1 == (calib.readCheckerboardParams(dx, dy, rows, cols))) {
        return 1;
    } else {
        return -1;
    }
}
}
