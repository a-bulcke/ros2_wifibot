#include "ybimu_ros2_driver/ybimu.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>

namespace ybimu {

YbImu::YbImu(const std::string & device, int baudrate) {
  if (!openPort(device, baudrate)) {
    std::cerr << "YbImu: impossible d'ouvrir " << device
              << ": " << std::strerror(errno) << std::endl;
  }
}

YbImu::~YbImu() {
  if (_fd >= 0) close(_fd);
}

bool YbImu::openPort(const std::string & device, int /*baudrate*/) {
  _fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (_fd < 0) return false;

  struct termios tty;
  if (tcgetattr(_fd, &tty) != 0) {
    close(_fd);
    _fd = -1;
    return false;
  }

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
    close(_fd);
    _fd = -1;
    return false;
  }

  tcflush(_fd, TCIOFLUSH);
  return true;
}

void YbImu::spinOnce() {
  if (_fd < 0) return;
  uint8_t buf[256];
  while (true) {
    ssize_t n = read(_fd, buf, sizeof(buf));
    if (n <= 0) break;
    for (ssize_t i = 0; i < n; ++i) {
      processByte(buf[i]);
    }
    if (static_cast<size_t>(n) < sizeof(buf)) break;
  }
}

bool YbImu::setReportRate(uint8_t rateHz) {
  if (_fd < 0) return false;
  if (rateHz < 10) rateHz = 10;
  if (rateHz > 100) rateHz = 100;

  std::vector<uint8_t> cmd = {HEAD1, HEAD2, 0x00, FUNC_REPORT_RATE, rateHz, 0x5F};
  cmd[2] = static_cast<uint8_t>(cmd.size() + 1);  // longueur totale = 7

  uint32_t checksum = 0;
  for (uint8_t b : cmd) checksum += b;
  cmd.push_back(static_cast<uint8_t>(checksum & 0xFF));

  ssize_t n = write(_fd, cmd.data(), cmd.size());
  return n == static_cast<ssize_t>(cmd.size());
}

void YbImu::processByte(uint8_t data) {
  switch (_rxState) {
    case 0:
      if (data == HEAD1) _rxState = 1;
      break;
    case 1:
      _rxState = (data == HEAD2) ? 2 : 0;
      break;
    case 2:
      _dataLen = data;
      _rxState = (_dataLen >= 5 && _dataLen <= RX_MAX_LEN) ? 3 : 0;
      break;
    case 3:
      _dataFunc = data;
      _rxData.clear();
      _rxState = 4;
      break;
    case 4: {
      _rxData.push_back(data);
      int dataBytesNeeded = static_cast<int>(_dataLen) - 5;
      if (static_cast<int>(_rxData.size()) >= dataBytesNeeded) {
        _rxState = 5;
      }
      break;
    }
    case 5: {
      uint8_t rxCheck = data;
      uint32_t checksum = HEAD1 + HEAD2 + _dataLen + _dataFunc;
      for (uint8_t b : _rxData) checksum += b;
      checksum &= 0xFF;
      if (rxCheck == checksum) {
        parseFrame();
      }
      _rxState = 0;
      break;
    }
    default:
      _rxState = 0;
      break;
  }
}

void YbImu::parseFrame() {
  auto s16 = [&](size_t i) -> int16_t {
    return static_cast<int16_t>(_rxData[i] | (_rxData[i + 1] << 8));
  };
  auto f32 = [&](size_t i) -> float {
    uint32_t raw = static_cast<uint32_t>(_rxData[i])
                 | (static_cast<uint32_t>(_rxData[i + 1]) << 8)
                 | (static_cast<uint32_t>(_rxData[i + 2]) << 16)
                 | (static_cast<uint32_t>(_rxData[i + 3]) << 24);
    float f;
    std::memcpy(&f, &raw, sizeof(f));
    return f;
  };

  switch (_dataFunc) {
    case FUNC_REPORT_IMU_RAW: {
      constexpr double ACCEL_RATIO = 16.0 / 32767.0;
      constexpr double GYRO_RATIO = (2000.0 / 32767.0) * (M_PI / 180.0);
      constexpr double MAG_RATIO = 800.0 / 32767.0;
      _imuRaw.ax = s16(0) * ACCEL_RATIO;
      _imuRaw.ay = s16(2) * ACCEL_RATIO;
      _imuRaw.az = s16(4) * ACCEL_RATIO;
      _imuRaw.gx = s16(6) * GYRO_RATIO;
      _imuRaw.gy = s16(8) * GYRO_RATIO;
      _imuRaw.gz = s16(10) * GYRO_RATIO;
      _imuRaw.mx = s16(12) * MAG_RATIO;
      _imuRaw.my = s16(14) * MAG_RATIO;
      _imuRaw.mz = s16(16) * MAG_RATIO;
      break;
    }
    case FUNC_REPORT_IMU_QUAT:
      _quat.w = f32(0);
      _quat.x = f32(4);
      _quat.y = f32(8);
      _quat.z = f32(12);
      break;
    case FUNC_REPORT_IMU_EULER:
      _euler.roll = f32(0);
      _euler.pitch = f32(4);
      _euler.yaw = f32(8);
      break;
    default:
      break;
  }
}

}  // namespace ybimu