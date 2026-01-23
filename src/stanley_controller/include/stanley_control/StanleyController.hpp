#ifndef STANLEY_CONTROLLER_HPP
#define STANLEY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/utils.h>
#include <vector>
#include <string>

struct Waypoint { 
    double x, y, v; 
};

class StanleyController : public rclcpp::Node {
public:
    StanleyController();

private:
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void lane_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void timer_callback();

    // Helpers
    std::vector<Waypoint> load_waypoints(const std::string& path);
    void publish_visuals(const Waypoint& lookahead, const Waypoint& closest, const std::string& frame_id);

    // ROS 2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lane_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;       // Target Point (Red)
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_pub_;      // Closest Point (Blue)
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_line_pub_;    // Full Path Line

    rclcpp::TimerBase::SharedPtr timer_;

    // Controller Data
    std::vector<std::vector<Waypoint>> lanes_;
    size_t current_lane_ = 0;
    
    // Stanley Parameters
    double k_e_, k_h_, v_scale_, wheelbase_;
};

#endif