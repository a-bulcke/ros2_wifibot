#ifndef YBIMU_ROS2_DRIVER_YBIMU_H
#define YBIMU_ROS2_DRIVER_YBIMU_H

#include <cstdint>
#include <string>
#include <vector>

namespace ybimu {

enum FuncWord : uint8_t {
  FUNC_VERSION          = 0x01,
  FUNC_REPORT_IMU_RAW   = 0x04,
  FUNC_REPORT_IMU_QUAT  = 0x16,
  FUNC_REPORT_IMU_EULER = 0x26,
  FUNC_REPORT_BARO      = 0x32,
  FUNC_REPORT_RATE      = 0x60,
};

struct ImuRawData {
  double ax = 0.0, ay = 0.0, az = 0.0;
  double gx = 0.0, gy = 0.0, gz = 0.0;
  double mx = 0.0, my = 0.0, mz = 0.0;
};

struct QuatData { double w = 1.0, x = 0.0, y = 0.0, z = 0.0; };
struct EulerData { double roll = 0.0, pitch = 0.0, yaw = 0.0; };

class YbImu {
public:
  explicit YbImu(const std::string & device, int baudrate = 115200);
  ~YbImu();

  bool isOpen() const { return _fd >= 0; }
  void spinOnce();

  const ImuRawData & imuRaw() const { return _imuRaw; }
  const QuatData & quat() const { return _quat; }
  const EulerData & euler() const { return _euler; }
  
  bool setReportRate(uint8_t rateHz);

private:
  bool openPort(const std::string & device, int baudrate);
  void processByte(uint8_t b);
  void parseFrame();

  int _fd = -1;
  int _rxState = 0;
  uint8_t _dataLen = 0;
  uint8_t _dataFunc = 0;
  std::vector<uint8_t> _rxData;

  static constexpr uint8_t HEAD1 = 0x7E;
  static constexpr uint8_t HEAD2 = 0x23;
  static constexpr size_t RX_MAX_LEN = 40;

  ImuRawData _imuRaw;
  QuatData _quat;
  EulerData _euler;
};

}  // namespace ybimu

#endif  // YBIMU_ROS2_DRIVER_YBIMU_H