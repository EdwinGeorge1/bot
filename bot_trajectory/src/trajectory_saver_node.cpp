#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <deque>
#include <fstream>
#include <iomanip>
#include <string>
#include <chrono>
#include <nlohmann/json.hpp>  // Add this dependency for JSON support

using json = nlohmann::json;

struct TimedPoint {
  rclcpp::Time timestamp;
  geometry_msgs::msg::Point point;
};

class TrajectorySaverNode : public rclcpp::Node {
public:
  TrajectorySaverNode() : Node("trajectory_saver_node") {
    // Declare parameters for save format and duration (seconds)
    this->declare_parameter<std::string>("save_format", "yaml");  // yaml, csv, json
    this->declare_parameter<double>("save_duration", 0.0);        // 0 means all

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TrajectorySaverNode::odomCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/trajectory_marker", 10);

    save_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/save_trajectory",
        std::bind(&TrajectorySaverNode::handleSaveRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

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
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    // Get parameters
    auto save_format = this->get_parameter("save_format").as_string();
    auto save_duration = this->get_parameter("save_duration").as_double();

    // Filter trajectory by duration if > 0
    std::deque<TimedPoint> filtered_trajectory;
    if (save_duration > 0 && !trajectory_.empty()) {
      auto latest_time = trajectory_.back().timestamp;
      for (auto it = trajectory_.rbegin(); it != trajectory_.rend(); ++it) {
        double dt = (latest_time - it->timestamp).seconds();
        if (dt <= save_duration) {
          filtered_trajectory.push_front(*it);
        } else {
          break;
        }
      }
    } else {
      filtered_trajectory = trajectory_;
    }

    try {
      if (save_format == "yaml") {
        saveYaml(filtered_trajectory);
      } else if (save_format == "csv") {
        saveCsv(filtered_trajectory);
      } else if (save_format == "json") {
        saveJson(filtered_trajectory);
      } else {
        throw std::runtime_error("Unsupported save_format parameter: " + save_format);
      }

      response->success = true;
      response->message = "Trajectory saved in " + save_format + " format";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("Failed to save trajectory: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  void saveYaml(const std::deque<TimedPoint>& traj) {
    std::ofstream file("trajectory.yaml");
    file << "trajectory:\n";
    for (const auto& tp : traj) {
      file << "  - time: " << std::fixed << std::setprecision(3) << tp.timestamp.seconds() << "\n";
      file << "    x: " << tp.point.x << "\n";
      file << "    y: " << tp.point.y << "\n";
      file << "    z: " << tp.point.z << "\n";
    }
  }

  void saveCsv(const std::deque<TimedPoint>& traj) {
    std::ofstream file("trajectory.csv");
    file << "time,x,y,z\n";
    for (const auto& tp : traj) {
      file << std::fixed << std::setprecision(3) << tp.timestamp.seconds() << ","
           << tp.point.x << "," << tp.point.y << "," << tp.point.z << "\n";
    }
  }

  void saveJson(const std::deque<TimedPoint>& traj) {
    json jtraj = json::array();
    for (const auto& tp : traj) {
      jtraj.push_back({
        {"time", tp.timestamp.seconds()},
        {"x", tp.point.x},
        {"y", tp.point.y},
        {"z", tp.point.z}
      });
    }
    json root;
    root["trajectory"] = jtraj;
    std::ofstream file("trajectory.json");
    file << std::setw(2) << root << std::endl;
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
