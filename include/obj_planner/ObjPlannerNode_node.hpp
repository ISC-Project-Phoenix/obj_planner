#pragma once

#include "obj_planner/IBackEnd.hpp"
#include "obj_planner/IFrontEnd.hpp"
#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"

class ObjPlannerNode : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tracks_sub;

    /// Creates a path from classified detections
    std::unique_ptr<IBackEnd> backend;
    /// Classifies connections into left and right
    std::unique_ptr<IFontEnd> frontEnd;

    void tracks_cb(geometry_msgs::msg::PoseArray::SharedPtr track);
    void calc_latency(long ms) const;

public:
    ObjPlannerNode(const rclcpp::NodeOptions& options);
};
