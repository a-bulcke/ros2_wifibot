#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "ybimu_ros2_driver/ybimu.h"

namespace {

// Produit de Hamilton q1 ⊗ q2 (convention w,x,y,z)
struct Quat { double w, x, y, z; };

Quat hamilton(const Quat & a, const Quat & b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
  };
}

}  // namespace

class YbImuNode : public rclcpp::Node {
public:
  YbImuNode() : Node("ybimu_node") {
    declare_parameter<std::string>("port", "/dev/myimu");
    declare_parameter<std::string>("frame_id", "imu_link");
    declare_parameter<int>("publish_period_ms", 40);   // aligné sur 25 Hz par défaut
    declare_parameter<int>("report_rate_hz", 25);       // débit demandé au module (10-100)
    declare_parameter<std::string>("mount_flip_axis", "none");  // none|x|y|z

    std::string port = get_parameter("port").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    int period_ms = static_cast<int>(get_parameter("publish_period_ms").as_int());
    int report_rate = static_cast<int>(get_parameter("report_rate_hz").as_int());
    std::string flip_axis = get_parameter("mount_flip_axis").as_string();

    setupMountCorrection(flip_axis);

    imu_ = std::make_unique<ybimu::YbImu>(port);
    if (!imu_->isOpen()) {
      RCLCPP_ERROR(get_logger(), "Impossible d'ouvrir le port IMU: %s", port.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "IMU YbImu ouverte sur %s", port.c_str());
      if (imu_->setReportRate(static_cast<uint8_t>(report_rate))) {
        RCLCPP_INFO(get_logger(), "Débit de sortie demandé: %d Hz", report_rate);
      } else {
        RCLCPP_WARN(get_logger(), "Echec de la commande set_report_rate");
      }
    }

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&YbImuNode::timerCallback, this));
  }

private:
  //   flip "x" : Y et Z inversés, X inchangé
  //   flip "y" : X et Z inversés, Y inchangé
  //   flip "z" : X et Y inversés, Z inchangé
  void setupMountCorrection(const std::string & axis) {
    flip_x_ = flip_y_ = flip_z_ = false;
    if (axis == "x") {
      flip_y_ = flip_z_ = true;
      mount_quat_ = {0.0, 1.0, 0.0, 0.0};
    } else if (axis == "y") {
      flip_x_ = flip_z_ = true;
      mount_quat_ = {0.0, 0.0, 1.0, 0.0};
    } else if (axis == "z") {
      flip_x_ = flip_y_ = true;
      mount_quat_ = {0.0, 0.0, 0.0, 1.0};
    } else {
      mount_quat_ = {1.0, 0.0, 0.0, 0.0};  // identité, pas de correction
    }
    RCLCPP_INFO(get_logger(), "Correction de montage: mount_flip_axis=%s", axis.c_str());
  }

  void timerCallback() {
    imu_->spinOnce();
    publishImu();
    publishMag();
  }

  void publishImu() {
    const auto & raw = imu_->imuRaw();
    const auto & q = imu_->quat();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    // Orientation : réinterprétation dans le repère corrigé (q_raw ⊗ q_mount)
    Quat qc = hamilton({q.w, q.x, q.y, q.z}, mount_quat_);
    msg.orientation.w = qc.w;
    msg.orientation.x = qc.x;
    msg.orientation.y = qc.y;
    msg.orientation.z = qc.z;

    msg.angular_velocity.x = flip_x_ ? -raw.gx : raw.gx;
    msg.angular_velocity.y = flip_y_ ? -raw.gy : raw.gy;
    msg.angular_velocity.z = flip_z_ ? -raw.gz : raw.gz;

    constexpr double G = 9.80665;
    msg.linear_acceleration.x = (flip_x_ ? -raw.ax : raw.ax) * G;
    msg.linear_acceleration.y = (flip_y_ ? -raw.ay : raw.ay) * G;
    msg.linear_acceleration.z = (flip_z_ ? -raw.az : raw.az) * G;

    msg.orientation_covariance.fill(0.0);
    msg.angular_velocity_covariance.fill(0.0);
    msg.linear_acceleration_covariance.fill(0.0);

    pub_imu_->publish(msg);
  }

  void publishMag() {
    const auto & raw = imu_->imuRaw();

    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    constexpr double UT_TO_T = 1e-6;
    msg.magnetic_field.x = (flip_x_ ? -raw.mx : raw.mx) * UT_TO_T;
    msg.magnetic_field.y = (flip_y_ ? -raw.my : raw.my) * UT_TO_T;
    msg.magnetic_field.z = (flip_z_ ? -raw.mz : raw.mz) * UT_TO_T;
    msg.magnetic_field_covariance.fill(0.0);

    pub_mag_->publish(msg);
  }

  std::unique_ptr<ybimu::YbImu> imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;

  bool flip_x_ = false, flip_y_ = false, flip_z_ = false;
  Quat mount_quat_ = {1.0, 0.0, 0.0, 0.0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YbImuNode>());
  rclcpp::shutdown();
  return 0;
}