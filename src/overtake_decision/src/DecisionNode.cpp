#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <algorithm>

class DecisionNode : public rclcpp::Node {
public:
    DecisionNode() : Node("decision_node"), current_lane_(0), consecutive_count_(0), last_parsed_direction_("NONE") {
        // Declare and get parameter
        this->declare_parameter("hysteresis_threshold", 5);
        threshold_ = this->get_parameter("hysteresis_threshold").as_int();

        // Subscribers & Publishers
        vlm_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/vlm_output", 10, std::bind(&DecisionNode::vlm_callback, this, std::placeholders::_1));
        
        lane_pub_ = this->create_publisher<std_msgs::msg::Int32>("/target_lane", 10);

        RCLCPP_INFO(this->get_logger(), "Overtake Decision Node (C++) Started. Waiting for VLM output...");
    }

private:
    void vlm_callback(const std_msgs::msg::String::SharedPtr msg) {
        // 1. Cleaning & Keyword Extraction
        std::string raw_text = msg->data;
        // Convert to uppercase
        std::transform(raw_text.begin(), raw_text.end(), raw_text.begin(), ::toupper);

        std::string clean_direction = "NONE";
        if (raw_text.find("RIGHT") != std::string::npos) {
            clean_direction = "RIGHT";
        } else if (raw_text.find("LEFT") != std::string::npos) {
            clean_direction = "LEFT";
        } else if (raw_text.find("CENTER") != std::string::npos) {
            clean_direction = "CENTER";
        }

        // 2. Hysteresis Filtering
        if (clean_direction == last_parsed_direction_ && clean_direction != "NONE") {
            consecutive_count_++;
        } else {
            consecutive_count_ = 0;
            last_parsed_direction_ = clean_direction;
        }

        // 3. Decision Logic Execution
        if (consecutive_count_ >= threshold_) {
            process_strategy(clean_direction);
        }
    }

    void process_strategy(const std::string& direction) {
        // Strategic mapping
        int target = 0; // Default to racing line (0)
        
        if (direction == "CENTER" || direction == "RIGHT") {
            target = 1; // Overtake via Left Lane
        } else if (direction == "LEFT") {
            target = 2; // Overtake via Right Lane
        }

        if (target != current_lane_) {
            current_lane_ = target;
            std_msgs::msg::Int32 msg;
            msg.data = current_lane_;
            lane_pub_->publish(msg);
            
            std::string lane_name;
            if (target == 0) lane_name = "CENTER";
            else if (target == 1) lane_name = "LEFT";
            else if (target == 2) lane_name = "RIGHT";

            RCLCPP_INFO(this->get_logger(), "STRATEGY UPDATE: Confirmed %s. Switching to %s lane.", direction.c_str(), lane_name.c_str());
        }
    }

    // Members
    int threshold_;
    int current_lane_;
    int consecutive_count_;
    std::string last_parsed_direction_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vlm_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lane_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
