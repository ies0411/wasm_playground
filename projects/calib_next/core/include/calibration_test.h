#include <iostream>
#include <memory>

class Calibration {
 public:
  virtual int getDecocdedData(const std::string file_name, std::unique_ptr<double> data, const std::string type) = 0;
  // virtual int getDataResolution() = 0;
  // virtual int readData() = 0;
  // virtual int removeData() = 0;
  // virtual int clearData() = 0;
  // virtual int getResult() = 0;
  virtual ~Calibration() = default;
};

// Calibration::Calibration(/* args */) {
// }
class CameraIntrinsic : public Calibration {
 public:
  int getDecocdedData(const std::string file_name, std::unique_ptr<double> data, const std::string type) override
};

// class SelectCalibType {
//  public:
//   // SelectCalibType(/* args */);
//   virtual ~SelectCalibType() = default;
// };

// Creator
class CharacterCreator {
 public:
  Calibration* Create() {
    return calibType();
  }
  virtual Calibration* calibType() = 0;
};

class CreateCameraIntrinsic : public CharacterCreator {
  Calibration* calibType() override {
    return new CameraIntrinsic;
  }
};
