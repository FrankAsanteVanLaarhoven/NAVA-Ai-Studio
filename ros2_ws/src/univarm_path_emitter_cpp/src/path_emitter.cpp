#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <univarm_msgs/srv/solve_path.hpp>
#include <univarm_msgs/msg/path_with_meta.hpp>

#include <grpcpp/grpcpp.h>
#include "drive.grpc.pb.h"

using univarm_msgs::srv::SolvePath;
using univarm_msgs::msg::PathWithMeta;

class PathEmitter : public rclcpp::Node {
public:
  PathEmitter() : Node("univarm_path_emitter") {
    this->declare_parameter<std::string>("planner_addr", "127.0.0.1:50051");
    planner_addr_ = this->get_parameter("planner_addr").as_string();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/univarm/drive/path", 10);
    meta_pub_ = this->create_publisher<PathWithMeta>("/univarm/drive/path_meta", 10);

    service_ = this->create_service<SolvePath>("/univarm/solve_path",
      std::bind(&PathEmitter::on_solve, this, std::placeholders::_1, std::placeholders::_2));

    // gRPC channel
    channel_ = grpc::CreateChannel(planner_addr_, grpc::InsecureChannelCredentials());
    stub_ = univarm::drive::Drive::NewStub(channel_);

    RCLCPP_INFO(this->get_logger(), "PathEmitter ready; planner_addr=%s", planner_addr_.c_str());
  }

private:
  void on_solve(const std::shared_ptr<SolvePath::Request> req,
                std::shared_ptr<SolvePath::Response> resp) {
    // Build proto request
    univarm::drive::SolvePathRequest preq;
    preq.mutable_start()->set_x(req->start.pose.position.x);
    preq.mutable_start()->set_y(req->start.pose.position.y);
    // yaw is unknown from PoseStamped here; keep zero
    preq.mutable_start()->set_yaw(0.0);

    preq.mutable_goal()->set_x(req->goal.pose.position.x);
    preq.mutable_goal()->set_y(req->goal.pose.position.y);
    preq.mutable_goal()->set_yaw(0.0);

    for (const auto & s : req->objectives) {
      preq.add_objectives(s);
    }

    // Call gRPC
    univarm::drive::SolvePathResponse presp;
    grpc::ClientContext ctx;
    grpc::Status status = stub_->SolvePath(&ctx, preq, &presp);

    if (!status.ok()) {
      RCLCPP_ERROR(this->get_logger(), "gRPC SolvePath failed: %s", status.error_message().c_str());
      resp->ok = false;
      return;
    }

    // Convert to ROS Path
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    PathWithMeta meta;
    meta.header = path.header;
    meta.length = static_cast<float>(presp.length());
    meta.risk_max = static_cast<float>(presp.risk_max());
    meta.planner_id = presp.planner_id();
    meta.preset = presp.preset();

    for (const auto & pt : presp.points()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pt.x();
      ps.pose.position.y = pt.y();
      ps.pose.position.z = 0.0;
      path.poses.push_back(ps);
      meta.poses.push_back(ps);
      meta.curvature.push_back(static_cast<float>(pt.curvature()));
    }

    // Publish
    path_pub_->publish(path);
    meta_pub_->publish(meta);

    // Fill response
    resp->ok = true;
    resp->path = path;
    resp->meta = meta;
  }

  std::string planner_addr_;
  rclcpp::Service<SolvePath>::SharedPtr service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<PathWithMeta>::SharedPtr meta_pub_;

  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<univarm::drive::Drive::Stub> stub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathEmitter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
