// Localization + velocity publisher (C++).
//
// Functionally identical to the Python localization_node: integrates a clamped
// Twist command through a unicycle model and converts the local ENU pose to
// WGS-84 latitude/longitude around a fixed origin near OvGU Magdeburg.
//
// Published topics:
//   /robot/gps  (sensor_msgs/msg/NavSatFix)
//   /cmd_vel    (geometry_msgs/msg/Twist)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kEarthRadiusM = 6378137.0;  // WGS-84 mean radius
constexpr double kPi = 3.14159265358979323846;

double clampSym(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}
}  // namespace

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode() : Node("localization_node")
  {
    lat0_ = this->declare_parameter<double>("center_lat", 52.139200);
    lon0_ = this->declare_parameter<double>("center_lon", 11.645200);
    rate_ = this->declare_parameter<double>("rate_hz", 30.0);
    max_lin_ = this->declare_parameter<double>("max_linear", 0.2);
    max_ang_ = this->declare_parameter<double>("max_angular", 0.2);
    dt_ = 1.0 / rate_;

    m_per_deg_lat_ = (kPi / 180.0) * kEarthRadiusM;
    m_per_deg_lon_ = m_per_deg_lat_ * std::cos(lat0_ * kPi / 180.0);

    // Best-effort sensor QoS for the GPS fix; reliable default for cmd_vel.
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();

    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/robot/gps", sensor_qos);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto period = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalizationNode::onTick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "localization_node up @ %.0f Hz, origin (%.6f, %.6f), limits lin<=%.2f ang<=%.2f",
      rate_, lat0_, lon0_, max_lin_, max_ang_);
  }

private:
  void demoCommand(double & linear, double & angular) const
  {
    // Drive forward at max speed, weaving slowly so the marker traces an arc.
    linear = max_lin_;
    angular = max_ang_ * std::sin(0.10 * t_);
  }

  void onTick()
  {
    t_ += dt_;

    double lin_cmd, ang_cmd;
    demoCommand(lin_cmd, ang_cmd);

    // Enforce the hard velocity envelope regardless of command source.
    const double v = clampSym(lin_cmd, max_lin_);
    const double w = clampSym(ang_cmd, max_ang_);

    // Unicycle integration.
    theta_ += w * dt_;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));
    x_ += v * std::cos(theta_) * dt_;
    y_ += v * std::sin(theta_) * dt_;

    const double lat = lat0_ + y_ / m_per_deg_lat_;
    const double lon = lon0_ + x_ / m_per_deg_lon_;

    const auto now = this->get_clock()->now();

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = now;
    fix.header.frame_id = "map";
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude = 55.0;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    gps_pub_->publish(fix);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = v;
    twist.angular.z = w;
    twist_pub_->publish(twist);
  }

  // Parameters / scale factors
  double lat0_, lon0_, rate_, max_lin_, max_ang_, dt_;
  double m_per_deg_lat_, m_per_deg_lon_;

  // Pose state
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0, t_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
