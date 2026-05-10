#ifndef SERVER_WIFIBOT_H
#define SERVER_WIFIBOT_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

// ✅ Include du message custom généré par ROS2
#include "ros2wifibot/msg/status.hpp"
namespace wifibot { class Driver; }

struct Position {
  double x;
  double y;
  double th;
};

class Wifibot : public rclcpp::Node {
public:
  Wifibot();
  virtual ~Wifibot();
  void update();

private:
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr vel);
  void computeOdometry(double dleft, double dright);
  double getSpeedLinear(double speedLeft, double speedRight);
  double getSpeedAngular(double speedLeft, double speedRight);

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         _pubOdometry;
  rclcpp::Publisher<ros2wifibot::msg::Status>::SharedPtr        _pubStatus;       // ✅ ajouté + bon namespace
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          _pubRobotBatteryVoltage;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          _pubComputerBatteryVoltage;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             _pubIsCharging;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    _subSpeeds;

  // TF2
  std::shared_ptr<tf2_ros::TransformBroadcaster>                _odomBroadcast;
  geometry_msgs::msg::TransformStamped                          _odomTf;

  // ✅ Driver correctement typé (plus de void*)
  wifibot::Driver* _pDriver;

  // Paramètres
  std::string  _frameBase;
  std::string  _frameOdom;
  Position     _position;
  double       _odometryLeftLast;
  double       _odometryRightLast;
  double       _entrax;
  bool         _updated;
  double       _speedLeft;
  double       _speedRight;
  double       _batteryMinVoltage;
  double       _batteryMaxVoltage;

  // Temps
  rclcpp::Time _timeCurrent;
  rclcpp::Time _timeLast;
};

#endif // SERVER_WIFIBOT_H