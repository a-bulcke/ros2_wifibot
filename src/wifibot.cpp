#include "ros2wifibot/libwifibot.h"
#include "ros2wifibot/wifibot.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>

#define TWOPI (M_PI * 2)
using BYTE = uint8_t;

// ============================================================
//  Constructeur
// ============================================================
Wifibot::Wifibot()
: Node("wifibot_node"), _updated(false), _speedLeft(0.0), _speedRight(0.0)
{
  // --- Paramètres ---
  this->declare_parameter<std::string>("port",                 "/dev/ttyS0");
  this->declare_parameter<std::string>("base_frame",           "base_frame");
  this->declare_parameter<std::string>("odom_frame",           "odom");
  this->declare_parameter<double>     ("entrax",               0.30);
  this->declare_parameter<bool>       ("relay1",               true);
  this->declare_parameter<bool>       ("relay2",               true);
  this->declare_parameter<bool>       ("relay3",               true);
  this->declare_parameter<double>     ("battery_min_voltage",  10.5);
  this->declare_parameter<double>     ("battery_max_voltage",  12.1);

  std::string dev = this->get_parameter("port").as_string();
  _frameBase       = this->get_parameter("base_frame").as_string();
  _frameOdom       = this->get_parameter("odom_frame").as_string();
  _entrax          = this->get_parameter("entrax").as_double();
  _batteryMinVoltage = this->get_parameter("battery_min_voltage").as_double();
  _batteryMaxVoltage = this->get_parameter("battery_max_voltage").as_double();

  RCLCPP_INFO(this->get_logger(), "Wifibot device: %s. Entrax: %0.3f",
              dev.c_str(), _entrax);

  // --- Driver bas niveau ---
  // ✅ _pDriver est maintenant wifibot::Driver* (plus de void*)
  _pDriver = new wifibot::Driver(dev);
  _pDriver->setRelays(
    this->get_parameter("relay1").as_bool(),
    this->get_parameter("relay2").as_bool(),
    this->get_parameter("relay3").as_bool()
  );
  _pDriver->loopControlSpeed(0.01);
  _pDriver->setPid(0.8, 0.45, 0.0);
  _pDriver->setTicsPerMeter(5312.0);

  // Lecture initiale de l'odométrie
  wifibot::driverData st = _pDriver->readData();
  _odometryLeftLast  = st.odometryLeft;
  _odometryRightLast = st.odometryRight;

  // --- Publishers ---
  _pubOdometry               = this->create_publisher<nav_msgs::msg::Odometry>   ("odom",                     1);
  _pubStatus                 = this->create_publisher<ros2wifibot::msg::Status>   ("status",                   1); // ✅ bon namespace
  _pubRobotBatteryVoltage    = this->create_publisher<std_msgs::msg::Float32>     ("robot_battery_voltage",    1);
  _pubComputerBatteryVoltage = this->create_publisher<std_msgs::msg::Float32>     ("computer_battery_voltage", 1);
  _pubIsCharging             = this->create_publisher<std_msgs::msg::Bool>        ("is_charging",              1);

  // --- Subscriber ---
  _subSpeeds = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    std::bind(&Wifibot::velocityCallback, this, std::placeholders::_1));

  // --- TF2 ---
  _odomBroadcast = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  _timeCurrent = this->now();
  _timeLast    = this->now();

  RCLCPP_INFO(this->get_logger(), "Wifibot node ready.");
}

// ============================================================
//  Destructeur
// ============================================================
Wifibot::~Wifibot() {
  delete _pDriver;
}

// ============================================================
//  Callback vitesse
// ============================================================
void Wifibot::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr vel) {
  _speedLeft  = vel->linear.x - (vel->angular.z * (_entrax / 2.0));
  _speedRight = vel->linear.x + (vel->angular.z * (_entrax / 2.0));
  _updated = true;
}

// ============================================================
//  Calcul odométrie
// ============================================================
void Wifibot::computeOdometry(double dleft, double dright) {
  double dL = dleft  - _odometryLeftLast;
  double dR = dright - _odometryRightLast;
  _odometryLeftLast  = dleft;
  _odometryRightLast = dright;

  double dCenter = (dL + dR) / 2.0;
  double dTheta  = (dR - dL) / _entrax;

  _position.x  += dCenter * cos(_position.th + dTheta / 2.0);
  _position.y  += dCenter * sin(_position.th + dTheta / 2.0);
  _position.th += dTheta;

  // Normalisation de l'angle entre -PI et PI
  while (_position.th >  M_PI) _position.th -= TWOPI;
  while (_position.th < -M_PI) _position.th += TWOPI;
}

// ============================================================
//  Vitesse linéaire et angulaire
// ============================================================
double Wifibot::getSpeedLinear(double speedLeft, double speedRight) {
  return (speedLeft + speedRight) / 2.0;
}

double Wifibot::getSpeedAngular(double speedLeft, double speedRight) {
  return (speedRight - speedLeft) / _entrax;
}

// ============================================================
//  Boucle principale
// ============================================================
void Wifibot::update() {
  if (_updated) {
    _pDriver->setSpeeds(_speedLeft, _speedRight);
    _updated = false;
  }

  wifibot::driverData st = _pDriver->readData();
  _timeCurrent = this->now();

  // Relays (pas dans driverData → appel séparé)
  bool r1, r2, r3;
  _pDriver->getRelays(r1, r2, r3);

  // Status
  ros2wifibot::msg::Status topicStatus;
  topicStatus.battery_level     = st.voltage;
  topicStatus.current           = st.current;
  topicStatus.adc1              = st.adc[0];   // ✅ tableau adc[4]
  topicStatus.adc2              = st.adc[1];
  topicStatus.adc3              = st.adc[2];
  topicStatus.adc4              = st.adc[3];
  topicStatus.speed_front_left  = st.speedFrontLeft;
  topicStatus.speed_front_right = st.speedFrontRight;
  topicStatus.odometry_left     = st.odometryLeft;
  topicStatus.odometry_right    = st.odometryRight;
  topicStatus.version           = st.version;
  topicStatus.relay1            = r1;           // ✅ via getRelays()
  topicStatus.relay2            = r2;
  topicStatus.relay3            = r3;
  _pubStatus->publish(topicStatus);

  // Odométrie + TF2 (inchangé)
  computeOdometry(st.odometryLeft, st.odometryRight);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, _position.th);

  _odomTf.header.stamp            = _timeCurrent;
  _odomTf.header.frame_id         = _frameOdom;
  _odomTf.child_frame_id          = _frameBase;
  _odomTf.transform.translation.x = _position.x;
  _odomTf.transform.translation.y = _position.y;
  _odomTf.transform.translation.z = 0.0;
  _odomTf.transform.rotation      = tf2::toMsg(q);
  _odomBroadcast->sendTransform(_odomTf);

  nav_msgs::msg::Odometry odometryTopic;
  odometryTopic.header.stamp          = _timeCurrent;
  odometryTopic.header.frame_id       = _frameOdom;
  odometryTopic.child_frame_id        = _frameBase;
  odometryTopic.pose.pose.position.x  = _position.x;
  odometryTopic.pose.pose.position.y  = _position.y;
  odometryTopic.pose.pose.position.z  = 0.0;
  odometryTopic.pose.pose.orientation = tf2::toMsg(q);
  odometryTopic.twist.twist.linear.x  = getSpeedLinear (st.speedFrontLeft, st.speedFrontRight);
  odometryTopic.twist.twist.angular.z = getSpeedAngular(st.speedFrontLeft, st.speedFrontRight);
  _pubOdometry->publish(odometryTopic);

  std_msgs::msg::Float32 batt;
  batt.data = st.voltage;
  _pubRobotBatteryVoltage->publish(batt);

  _timeLast = _timeCurrent;
}

// ============================================================
//  main
// ============================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Wifibot>();
  rclcpp::Rate r(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    node->update();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}