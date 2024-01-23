#pragma once

#include "obj_planner/IBackEnd.hpp"
#include "obj_planner/IFrontEnd.hpp"
#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ObjPlannerNode : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tracks_sub;

    /// Creates a path from classified detections
    std::unique_ptr<IBackEnd> backend;
    /// Classifies connections into left and right
    std::unique_ptr<IFrontEnd> frontEnd;

    // TF2 stuff
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;

    void tracks_cb(geometry_msgs::msg::PoseArray::SharedPtr track);
    void calc_latency(long ms) const;

public:
    ObjPlannerNode(const rclcpp::NodeOptions& options);
};
