#include "stanley_control/StanleyController.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

StanleyController::StanleyController() : Node("stanley_controller") {
    // Declare Parameters matching your config structure
    std::string share_dir = ament_index_cpp::get_package_share_directory("stanley_control");
    this->declare_parameter("center_csv", share_dir + "/racelines/center.csv");
    this->declare_parameter("left_csv", share_dir + "/racelines/left.csv");
    this->declare_parameter("right_csv", share_dir + "/racelines/right.csv");
    this->declare_parameter("k_e", 1.5);
    this->declare_parameter("k_h", 1.0);
    this->declare_parameter("v_scale", 0.8);
    this->declare_parameter("wheelbase", 0.33);

    // Load Initial Parameter Values
    k_e_ = this->get_parameter("k_e").as_double();
    k_h_ = this->get_parameter("k_h").as_double();
    v_scale_ = this->get_parameter("v_scale").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();

    // Load CSVs into memory (0: Center, 1: Left, 2: Right)
    // Load CSVs into memory
    // User requested RIGHT lane as MAIN (Index 0)
    lanes_.push_back(load_waypoints(this->get_parameter("right_csv").as_string())); // Index 0: Right (Default/Main)
    lanes_.push_back(load_waypoints(this->get_parameter("left_csv").as_string()));  // Index 1: Left

    // ROS 2 Setup
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/pf/pose/odom", 1, std::bind(&StanleyController::odom_callback, this, std::placeholders::_1));
    lane_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/target_lane", 10, std::bind(&StanleyController::lane_callback, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_waypoint_viz", 10);
    
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StanleyController::timer_callback, this));
}

void StanleyController::timer_callback() {
    // Dynamically update gains from parameter server
    k_e_ = this->get_parameter("k_e").as_double();
    k_h_ = this->get_parameter("k_h").as_double();
    v_scale_ = this->get_parameter("v_scale").as_double();
}

void StanleyController::lane_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data >= 0 && static_cast<size_t>(msg->data) < lanes_.size()) {
        current_lane_ = static_cast<size_t>(msg->data);
    }
}

std::vector<Waypoint> StanleyController::load_waypoints(const std::string& path) {
    std::vector<Waypoint> wps;
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open CSV: %s", path.c_str());
        return wps;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) row.push_back(std::stod(val));
        if (row.size() >= 3) wps.push_back({row[0], row[1], row[2]});
    }
    return wps;
}

void StanleyController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto& path = lanes_[current_lane_];
    if (path.empty()) return;

    // 1. Get State
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    double v = std::max(msg->twist.twist.linear.x, 0.5); // Prevent division by zero

    // 2. Project to Front Axle
    double fx = x + wheelbase_ * cos(yaw);
    double fy = y + wheelbase_ * sin(yaw);

    // 3. Find Nearest Point
    size_t idx = 0;
    double min_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
        double d = std::hypot(path[i].x - fx, path[i].y - fy);
        if (d < min_d) { min_d = d; idx = i; }
    }

    // 4. Calculate Heading Error
    size_t next_idx = (idx + 1) % path.size();
    double path_yaw = std::atan2(path[next_idx].y - path[idx].y, path[next_idx].x - path[idx].x);
    double yaw_err = path_yaw - yaw;
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    // 5. Calculate Crosstrack Error (Lateral distance in car frame)
    double dx = fx - path[idx].x;
    double dy = fy - path[idx].y;
    double cte = -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);

    // 6. Stanley Control Law
    double steer = (k_h_ * yaw_err) + std::atan2(k_e_ * cte, v);

    // 7. Output Drive Message
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = this->now();
    drive.drive.speed = path[idx].v * v_scale_;
    drive.drive.steering_angle = std::clamp(steer, -0.44, 0.44); // ~25 deg limit
    drive_pub_->publish(drive);

    publish_visuals(path[idx], msg->header.frame_id);
}

void StanleyController::publish_visuals(const Waypoint& target, const std::string& frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = target.x;
    marker.pose.position.y = target.y;
    marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
    marker.color.a = 1.0; marker.color.g = 1.0; // Green
    marker_pub_->publish(marker);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyController>());
    rclcpp::shutdown();
    return 0;
}