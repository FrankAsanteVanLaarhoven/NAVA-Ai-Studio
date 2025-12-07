#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using std::placeholders::_1;

class CurvatureFollower : public rclcpp::Node {
public:
  CurvatureFollower()
  : Node("curvature_follower")
  {
    this->declare_parameter("v_max", 0.5);
    this->declare_parameter("w_max", 1.5);
    this->declare_parameter("lookahead", 0.6);
    this->declare_parameter("min_v", 0.1);
    this->declare_parameter("frame_id", "base_link");
    this->declare_parameter("risk_topic", "/univarm/occupancy_risk");

    v_max_ = this->get_parameter("v_max").as_double();
    w_max_ = this->get_parameter("w_max").as_double();
    lookahead_ = this->get_parameter("lookahead").as_double();
    min_v_ = this->get_parameter("min_v").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/univarm/drive/path", 10, std::bind(&CurvatureFollower::on_path, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), 
      std::bind(&CurvatureFollower::on_tick, this));
  }

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
    index_ = 0;
  }

  void on_tick() {
    if (path_.poses.empty()) return;

    // Fake pose at origin (replace with TF lookup from odom->base_link if available)
    double x = 0.0, y = 0.0, yaw = 0.0;

    // find lookahead target
    int i = index_;
    geometry_msgs::msg::PoseStamped best = path_.poses[i];
    auto dist = [&](const geometry_msgs::msg::PoseStamped& a) {
      double dx = a.pose.position.x - x;
      double dy = a.pose.position.y - y;
      return std::sqrt(dx*dx + dy*dy);
    };
    while (i + 1 < (int)path_.poses.size() && dist(path_.poses[i]) < lookahead_) {
      i++;
      best = path_.poses[i];
    }
    index_ = i;

    // transform target into base frame (assuming base at origin with yaw=0 for simplicity)
    double tx = best.pose.position.x - x;
    double ty = best.pose.position.y - y;

    double ld2 = std::max(1e-4, tx*tx + ty*ty);
    double ld = std::sqrt(ld2);
    double kappa = 2.0 * ty / ld2;

    // risk limiter (stub: constant 0)
    double risk = 0.0;
    double scale = std::clamp(1.0 - risk, 0.2, 1.0);

    double v = std::max(min_v_, std::min(v_max_ * scale, v_max_));
    double w = std::clamp(kappa * v, -w_max_, w_max_);

    if (std::abs(w) >= w_max_ - 1e-6) {
      v = std::clamp(w_max_ / std::abs(kappa), min_v_, v_max_);
      w = std::clamp(kappa * v, -w_max_, w_max_);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  int index_ {0};

  double v_max_, w_max_, lookahead_, min_v_;
  std::string frame_id_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurvatureFollower>());
  rclcpp::shutdown();
  return 0;
}
