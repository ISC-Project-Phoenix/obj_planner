#include "obj_planner/ObjPlannerNode_node.hpp"

#include "obj_planner/ConvexMethod.hpp"
#include "obj_planner/SimpleBackEnd.hpp"

// For _1
using namespace std::placeholders;

ObjPlannerNode::ObjPlannerNode(const rclcpp::NodeOptions& options) : Node("ObjPlannerNode", options) {
    // Random params
    this->declare_parameter("test_latency", false);

    this->tracks_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/tracks", 5, std::bind(&ObjPlannerNode::tracks_cb, this, _1));
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // Initialize strategies
    this->backend = std::make_unique<SimpleBackEnd>(SimpleBackEndParams{});  // TODO make param
    this->frontEnd = std::make_unique<ConvexMethod>(ConvexMethodParams{});   // TODO make param
}

void ObjPlannerNode::tracks_cb(geometry_msgs::msg::PoseArray::SharedPtr track) {
    auto start_t = std::chrono::steady_clock::now();

    //Run the pipeline
    auto classified = this->frontEnd->classify(*track);

    // If you only see one side of road, skip
    if (!classified.has_value()) {
        RCLCPP_INFO(this->get_logger(), "Only one side of the road visible, skipping");
        return;
    }

    auto path = this->backend->create_path(*classified, track->header.frame_id);

    if (path) {
        path.value().header.stamp = this->get_clock()->now();

        // TODO: sort this path prior to publishing
        // TODO convert from current frame (same as track) to odom
        this->path_pub->publish(*path);
    }

    if (this->get_parameter("test_latency").as_bool()) {
        auto delta = std::chrono::steady_clock::now() - start_t;
        auto ms = std::chrono::duration_cast<std::chrono::nanoseconds>(delta).count();
        calc_latency(ms);
    }
}

void ObjPlannerNode::calc_latency(long ms) const {
    static std::array<uint64_t, 300> measurements;
    static uint64_t index = 0;
    measurements[index] = ms;
    index = index + 1 > measurements.size() - 1 ? 0 : index + 1;

    // Calc statistics
    double mean = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean += (double)measurements[i];
    }
    mean /= (double)index + 1;

    double mean2 = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean2 += std::pow((double)measurements[i], 2);
    }
    mean2 /= (double)index + 1;

    double std_dev = sqrt(mean2 - std::pow(mean, 2));

    RCLCPP_INFO(get_logger(), "Mean: %fms Std-dev: %fms", mean * 1e-6, std_dev * 1e-6);
}
