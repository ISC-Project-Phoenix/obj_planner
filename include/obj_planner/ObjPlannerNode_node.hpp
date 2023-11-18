#pragma once

#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct LeftRightResults {
    std::vector<cv::Point2d> left_cones;
    std::vector<cv::Point2d> right_cones;
};

class ObjPlannerNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

public:
    ObjPlannerNode(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void sub_cb(std_msgs::msg::String::SharedPtr msg);
};
