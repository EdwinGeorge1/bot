#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <deque>
#include <fstream>
#include <iomanip>

struct TimedPoint {
  rclcpp::Time timestamp;
  geometry_msgs::msg::Point point;
};

class TrajectorySaverNode : public rclcpp::Node {
public:
  TrajectorySaverNode() : Node("trajectory_saver_node") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TrajectorySaverNode::odomCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/trajectory_marker", 10);

    save_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/save_trajectory",
        std::bind(&TrajectorySaverNode::handleSaveRequest, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&TrajectorySaverNode::publishMarkers, this));

    RCLCPP_INFO(this->get_logger(), "TrajectorySaverNode started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    TimedPoint tp{msg->header.stamp, msg->pose.pose.position};
    trajectory_.push_back(tp);
  }

  void publishMarkers() {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "odom";
    line.header.stamp = this->get_clock()->now();
    line.ns = "trajectory";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    for (const auto& tp : trajectory_) {
      line.points.push_back(tp.point);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(line);
    marker_pub_->publish(marker_array);
  }

  void handleSaveRequest(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;  // unused

    std::string filename = "trajectory.yaml";
    try {
      std::ofstream file(filename);
      file << "trajectory:\n";
      for (const auto& tp : trajectory_) {
        file << "  - time: " << std::fixed << std::setprecision(3) << tp.timestamp.seconds() << "\n";
        file << "    x: " << tp.point.x << "\n";
        file << "    y: " << tp.point.y << "\n";
        file << "    z: " << tp.point.z << "\n";
      }

      response->success = true;
      response->message = "Trajectory saved to " + filename;
      RCLCPP_INFO(this->get_logger(), "Saved trajectory to %s", filename.c_str());

    } catch (const std::exception &e) {
      response->success = false;
      response->message = "Failed to save file: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "File save error: %s", e.what());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<TimedPoint> trajectory_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySaverNode>());
  rclcpp::shutdown();
  return 0;
}
