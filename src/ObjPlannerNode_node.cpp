#include "obj_planner/ObjPlannerNode_node.hpp"

#include "obj_planner/IBackEnd.hpp"
#include "obj_planner/IFrontEnd.hpp"

// For _1
using namespace std::placeholders;

ObjPlannerNode::ObjPlannerNode(const rclcpp::NodeOptions& options) : Node("ObjPlannerNode", options) {
    // Parameters
    float x = this->declare_parameter<float>("foo", -10.0);

    // Pub Sub
    this->sub =
        this->create_subscription<std_msgs::msg::String>("/str", 1, std::bind(&ObjPlannerNode::sub_cb, this, _1));
    this->pub = this->create_publisher<std_msgs::msg::String>("/run_folder", 1);

    // Log a sample log
    RCLCPP_INFO(this->get_logger(), "You passed %f", x);

    // Send a sample message
    std_msgs::msg::String msg{};
    msg.data = std::string{"Hello World!"};
    pub->publish(msg);
}

void ObjPlannerNode::sub_cb(const std_msgs::msg::String::SharedPtr msg) {
    // Echo message
    this->pub->publish(*msg);
}
