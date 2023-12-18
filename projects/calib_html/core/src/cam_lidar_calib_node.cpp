#include "cam_lidar_calib.h"

camLidarCalib cLC;

extern "C" {
int EMSCRIPTEN_KEEPALIVE getDecodedPCD(char *file_name, double *decodedPCD) {
    std::string file_name_cpp(file_name);
    auto index = cLC.pcds_.find(file_name_cpp);
    if (index == cLC.pcds_.end()) {
        std::cout << "remove : can not find the file name" << std::endl;
        return -1;
    }

    for (int i = 0; i < index->second->points.size(); i += cLC.channel_) {
        decodedPCD[i] = index->second->points[i].x;
        decodedPCD[i + 1] = index->second->points[i].y;
        decodedPCD[i + 2] = index->second->points[i].z;
    }
    return 1;
}

int EMSCRIPTEN_KEEPALIVE getDecodedImage(char *file_name, uchar *decodedImage) {
    std::string file_name_cpp(file_name);
    auto index = cLC.images_.find(file_name_cpp);
    if (index == cLC.images_.end()) {
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

int EMSCRIPTEN_KEEPALIVE preSolveCalibPCDs(char *file_name) {
    std::string file_name_cpp(file_name);
    auto index = cLC.pcds_.find(file_name_cpp);
    if (index == cLC.pcds_.end()) {
        std::cout << "can not find the file name" << std::endl;
        return -1;
    }
    if (cLC.cloudHandler(index->second)) {
        return 1;
    } else {
        return -1;
    }
}
int EMSCRIPTEN_KEEPALIVE preSolveCalibImages(char *file_name) {
    std::string file_name_cpp(file_name);
    auto index = cLC.images_.find(file_name_cpp);
    if (index == cLC.images_.end()) {
        std::cout << "can not find the file name" << std::endl;
        return -1;
    }
    if (cLC.imageHandler(index->second)) {
        return 1;
    } else {
        return -1;
    }
}

int EMSCRIPTEN_KEEPALIVE solveCalib(double *result) {
    if (!cLC.solveWithPCL()) {
        return -1;
    }
    std::cout << "fisnish calib" << std::endl;
    for (int row = 0; row < cLC.rotation_matrix_.rows(); row++) {
        for (int col = 0; col < cLC.rotation_matrix_.cols(); col++) {
            result[(row + 1) * col] = cLC.rotation_matrix_(row, col);
        }
    }
    result[9] = cLC.translation_vector_(0);
    result[10] = cLC.translation_vector_(1);
    result[11] = cLC.translation_vector_(2);

    return 1;
}
int EMSCRIPTEN_KEEPALIVE getImageResolution(char *file_name, int *width, int *height) {
    std::string file_name_cpp(file_name);
    auto index = cLC.images_.find(file_name_cpp);
    if (index == cLC.images_.end()) {
        std::cout << "can not find the file name" << std::endl;
        return -1;
    }
    *width = index->second.cols;
    *height = index->second.rows;

    return 1;
}

int EMSCRIPTEN_KEEPALIVE readImage(char *file_name) {
    std::string file_name_cpp(file_name);
    std::cout << file_name_cpp << std::endl;
    cv::Mat img = cv::imread(file_name);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    std::string debug_file_name = "/debug.jpg";
    cv::imwrite(debug_file_name, gray);
    cLC.images_.insert({file_name_cpp, gray});

    std::cout << gray.rows << "," << gray.cols << std::endl;
    return 1;
}

int EMSCRIPTEN_KEEPALIVE readPCD(char *file_name) {
    std::string file_name_cpp(file_name);
    std::cout << file_name_cpp << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud)) {
        std::cout << "can not load pcd" << std::endl;
        return -1;
    }
    if (cloud->width * cloud->height == 0) {
        std::cout << "empty in pcd file" << std::endl;
        return -1;
    }
    cLC.pcds_.insert({file_name_cpp, cloud});
    std::cout << cloud->width << "," << cloud->height << std::endl;

    return 1;
}

int EMSCRIPTEN_KEEPALIVE removeImage(char *file_name) {
    std::string file_name_cpp(file_name);
    auto index = cLC.images_.find(file_name_cpp);
    if (index == cLC.images_.end()) {
        std::cout << "can not find the file name" << std::endl;
        return -1;
    }
    cLC.images_.erase(index);
    return 1;
}
int EMSCRIPTEN_KEEPALIVE removePCD(char *file_name) {
    std::string file_name_cpp(file_name);
    auto index = cLC.pcds_.find(file_name_cpp);
    if (index == cLC.pcds_.end()) {
        std::cout << "can not find the file name" << std::endl;
        return -1;
    }
    cLC.pcds_.erase(index);
    return 1;
}

int EMSCRIPTEN_KEEPALIVE clearImages() {
    cLC.images_.clear();
    std::map<std::string, cv::Mat>().swap(cLC.images_);
    return 1;
}
int EMSCRIPTEN_KEEPALIVE clearPCDs() {
    cLC.pcds_.clear();
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(cLC.pcds_);
    return 1;
}

int EMSCRIPTEN_KEEPALIVE getPCDSize(char *file_name, int *size, int *channel) {
    std::string file_name_cpp(file_name);
    auto index = cLC.pcds_.find(file_name_cpp);
    if (index == cLC.pcds_.end()) {
        std::cout << "can not find the file name" << std::endl;
        return -1;
    }
    *size = index->second->width * index->second->height;
    *channel = cLC.channel_;
    return 1;
}

int EMSCRIPTEN_KEEPALIVE readCheckerboardParams(double dx, double dy, int rows, int cols) {
    if (1 == (cLC.readCheckerboardParams(dx, dy, rows, cols))) {
        return 1;
    } else {
        return -1;
    }
}

int EMSCRIPTEN_KEEPALIVE readCameraParams(double *camera_info) {
    if (1 == (cLC.readCameraParams(camera_info))) {
        return 1;
    } else {
        return -1;
    }

    return 1;
}

int EMSCRIPTEN_KEEPALIVE readCheckerPosition(double *filter_info) {
    if (1 == (cLC.readCheckerPosition(filter_info))) {
        return 1;
    } else {
        return -1;
    }

    return 1;
}
}
