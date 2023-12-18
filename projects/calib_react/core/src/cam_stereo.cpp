#include "cam_stereo.h"
setStereoCalibEnv calib;

extern "C" {
int EMSCRIPTEN_KEEPALIVE preSolveStereoCalib() {
    int size = calib.images_left_.size();
    if (size == calib.images_right_.size()) {
        for (int i = 0; i < size; i++) {
            calib.preCalibStereoRawimage(calib.images_left_[i], calib.images_right_[i]);
        }
    } else {
        std::cout << "no of right and lef is not equal" << std::endl;
    }
}
int EMSCRIPTEN_KEEPALIVE SolveStereoCalib(double *left_result, double *right_result, double *RT) {
    calib.CalibStereoRawimage();
}
int EMSCRIPTEN_KEEPALIVE readImg(char *left_file_name, char *right_file_name) {
    cv::Mat left_img = cv::imread(left_file_name);
    cv::Mat right_img = cv::imread(right_file_name);

    cv::Mat left_gray, right_gray;
    cv::cvtColor(left_img, left_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(right_img, right_gray, cv::COLOR_RGB2GRAY);

    calib.images_left_.push_back(left_gray);
    calib.images_right_.push_back(right_gray);

    return 1;
}

int EMSCRIPTEN_KEEPALIVE readStereoCheckerboardParams(double dx, double dy, int rows, int cols) {
    if (1 == (calib.readCheckerboardParams(dx, dy, rows, cols))) {
        return 1;
    } else {
        return -1;
    }
}
}
