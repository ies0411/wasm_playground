#include "calibration.h"
extern "C" {
std::unique_ptr<FacadeCalib> calib = std::make_unique<FacadeCalib>();
// TODO : jpg
int EMSCRIPTEN_KEEPALIVE intrinsicReadCheckerboardParams(const double dx, const double dy, const int rows, const int cols) {
  if (!calib->intrinsicReadCheckerboardParams(std::move(dx), std::move(dy), std::move(rows), std::move(cols))) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE intrinsicGetImageResolution(const char* file_name, int* res) {
  std::vector<int32_t> res_vector(2, 0);
  std::string file_name_cpp(file_name);

  if (!calib->intrinsicGetImageResolution(std::move(file_name_cpp), res_vector)) {
    return -1;
  }

  res[0] = res_vector[0];
  res[1] = res_vector[1];

  return 1;
}
int EMSCRIPTEN_KEEPALIVE intrinsicReadImage(const char* file_name) {
  const std::string file_name_cpp(file_name);
  if (!calib->intrinsicReadImage(std::move(file_name_cpp))) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE intrinsicGetDecodedImage(const char* file_name, uchar* decoded_image) {
  const std::string file_name_cpp(file_name);
  std::vector<uchar> decoded_image_vector;
  if (!calib->intrinsicGetDecodedImage(std::move(file_name_cpp), decoded_image_vector)) {
    return -1;
  }
  uint16_t idx = 0;
  for (auto& p : decoded_image_vector) {
    decoded_image_vector[idx++] = p;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE intrinsicClear() {
  if (!calib->intrinsicClear()) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE intrinsicGetCalibResult(const char* file_name, double* R_result, double* T_result, double* error_value) {
  const std::string file_name_cpp(file_name);
  std::vector<double> R_result_vector;
  R_result_vector.reserve(3);
  std::vector<double> T_result_vector;
  T_result_vector.reserve(3);
  std::vector<double> error_value_vector;

  if (!calib->intrinsicGetCalibResult(std::move(file_name_cpp), R_result_vector, T_result_vector, error_value_vector)) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE intrinsicPreprocess(const char* file_name, uchar* debug_image) {
  const std::string file_name_cpp(file_name);
  std::vector<uchar> debug_image_vector;
  if (!calib->intrinsicPreprocess(std::move(file_name), debug_image_vector)) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE intrinsicRemoveImage(const char* file_name) {
  const std::string file_name_cpp(file_name);
  if (!calib->intrinsicRemoveImage(std::move(file_name))) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE intrinsicSolveCalib(double* result) {
  std::vector<double> result_vector;
  result_vector.reserve(9);
  if (!calib->intrinsicSolveCalib(result_vector)) {
    return -1;
  }
  uint16_t idx = 0;
  for (auto& p : result_vector) {
    result[idx++] = p;
  }
  return 1;
}
////////////

// int EMSCRIPTEN_KEEPALIVE cameraLidarClear() {
//   if (!calib->cameraLidarClear()) {
//     return -1;
//   }
//   return 1;
// }

int EMSCRIPTEN_KEEPALIVE cameraLidarGetDecodedPCD(const char* file_name, double* decodedPCD) {
  const std::string file_name_cpp(file_name);
  std::vector<double> decodedPCD_vector;
  if (!calib->cameraLidargetDecodedData(std::move(file_name_cpp), decodedPCD_vector)) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarGetDecodedImage(const char* file_name, uchar* decoded_image) {
  const std::string file_name_cpp(file_name);
  std::vector<uchar> decoded_image_vector;
  if (!calib->cameraLidargetDecodedData(std::move(file_name_cpp), decoded_image_vector)) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE cameraLidarPreprocess(const char* file_pcd_name, char* file_image_name) {
  const std::string file_pcd_name_cpp(file_pcd_name);
  const std::string file_image_name_cpp(file_image_name);
  if (!calib->cameraLidarPreprocess(std::move(file_pcd_name_cpp), std::move(file_image_name_cpp))) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE cameraLidarSolveCalib(double* result) {
  std::vector<double> result_vector;
  if (!calib->cameraLidarSolveCalib(result_vector)) {
    return -1;
  }
  uint16_t idx = 0;
  for (auto& p : result_vector) {
    result[idx++] = p;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarGetImageResolution(const char* file_name, int* width, int* height) {
  const std::string file_name_cpp(file_name);
  std::vector<int32_t> res;
  if (!calib->cameraLidarGetResultion(std::move(file_name_cpp), res, "image")) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarGetPCDSize(const char* file_name, int* size, int* chaanel) {
  const std::string file_name_cpp(file_name);
  std::vector<int32_t> res;
  if (!calib->cameraLidarGetResultion(std::move(file_name_cpp), res, "pcd")) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE cameraLidarReadImage(const char* file_name) {
  const std::string file_name_cpp(file_name);

  if (!calib->cameraLidarReadData(std::move(file_name_cpp), "image")) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarReadPCD(const char* file_name) {
  const std::string file_name_cpp(file_name);

  if (!calib->cameraLidarReadData(std::move(file_name_cpp), "pcd")) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarRemoveImage(const char* file_name) {
  const std::string file_name_cpp(file_name);

  if (!calib->cameraLidarRemoveData(std::move(file_name_cpp), "image")) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarRemovePCD(const char* file_name) {
  const std::string file_name_cpp(file_name);

  if (!calib->cameraLidarRemoveData(std::move(file_name_cpp), "pcd")) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarClearPCD() {
  if (!calib->cameraLidarClearData("pcd")) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE cameraLidarClearImage() {
  if (!calib->cameraLidarClearData("image")) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE cameraLidarReadCheckerboardParams(const double dx, const double dy, const int rows, const int cols) {
  if (!calib->cameraLidarReadCheckerboardParams(std::move(dx), std::move(dy), std::move(rows), std::move(cols))) {
    return -1;
  }
  return 1;
}
// TODO : double -> float (only param)
int EMSCRIPTEN_KEEPALIVE cameraLidarReadCheckerboardPosition(const double* filter_info) {
  std::vector<double> filter_vector(6, 0);
  uint8_t idx = 0;
  for (auto& p : filter_vector) {
    p = filter_info[idx++];
  }
  if (!calib->cameraLidarReadCheckerboardPosition(std::move(filter_vector))) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE cameraLidarReadCameraInfo(const double* camera_info) {
  std::vector<double> filter_vector(11, 0);
  uint8_t idx = 0;
  for (auto& p : filter_vector) {
    p = camera_info[idx++];
  }
  if (!calib->cameraLidarReadCameraInfo(std::move(filter_vector))) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE stereoReadImage(const char* left_name, const char* right_name) {
  const std::string left_name_cpp(left_name);
  const std::string right_name_cpp(right_name);

  if (!calib->stereoReadImage(std::move(left_name_cpp), std::move(right_name_cpp))) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE stereoReadCheckerboardParams(double dx, double dy, int rows, int cols) {
  if (!calib->stereoReadCheckerboardParams(std::move(dx), std::move(dy), std::move(rows), std::move(cols))) {
    return -1;
  }
  return 1;
}

int EMSCRIPTEN_KEEPALIVE stereoPreprocess(const char* left_name, const char* right_name, uchar* debug_image) {
  const std::string left_name_cpp(left_name);
  const std::string right_name_cpp(right_name);
  std::vector<uchar> debug_vector;
  if (!calib->stereoPreprocess(std::move(left_name_cpp), std::move(right_name_cpp), debug_vector)) {
    return -1;
  }
  return 1;
}
int EMSCRIPTEN_KEEPALIVE stereoSolveCalib(double* result) {
  std::vector<double> result_vector;
  // result_vector.reserve(9);
  if (!calib->stereoSolveCalib(result_vector)) {
    return -1;
  }
  uint16_t idx = 0;
  for (auto& p : result_vector) {
    result[idx++] = p;
  }
  return 1;
}
//////////////
// intrinsicSolveCalib
// virtual bool preSolveCalib(const std::string&& file_name, std::vector<uchar>& debug_image) override;

// virtual bool solveCalib(std::vector<double> &result) override;
}
int main() {
  return 1;
}
