#include "obj_planner/ObjPlannerNode_node.hpp"

#include "obj_planner/ConvexMethod.hpp"
#include "obj_planner/SimpleBackEnd.hpp"
#include "opencv2/highgui.hpp"

// Required for doTransform
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// For _1
using namespace std::placeholders;

ObjPlannerNode::ObjPlannerNode(const rclcpp::NodeOptions& options) : Node("ObjPlannerNode", options) {
    // Random params
    this->declare_parameter("test_latency", false);
    auto debug = this->declare_parameter("debug", true);

    // Strategy params
    ConvexMethodParams cm_p{};
    cm_p.debug = debug;

    // Frame params
    this->declare_parameter("path_frame", "odom");

    this->tracks_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/tracks", 5, std::bind(&ObjPlannerNode::tracks_cb, this, _1));
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);

    // Initialize strategies
    this->backend = std::make_unique<SimpleBackEnd>(SimpleBackEndParams{});     // TODO make param
    this->frontEnd = std::make_unique<ConvexMethod>(cm_p, this->get_logger());  // TODO make param

    if (debug) {
        cv::namedWindow("Hull");
    }
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

        try {
            // Transform path to odom. While the path is in the same location, its now WRT where the kart started, which
            // means that as the kart moves and this path is transformed back WRT the kart, the path will have moved location.
            auto trans = this->tf2_buffer->lookupTransform(this->get_parameter("path_frame").as_string(),
                                                           path->header.frame_id, rclcpp::Time{});
            for (auto& pose : path.value().poses) {
                tf2::doTransform(pose, pose, trans);
                pose.header.frame_id = this->get_parameter("path_frame").as_string();
                pose.header.stamp = this->get_clock()->now();
            }
            path.value().header.frame_id = this->get_parameter("path_frame").as_string();
        } catch (tf2::LookupException& e) {
            RCLCPP_INFO(this->get_logger(), "Could not look up odom!");
            return;
        }

        this->path_pub->publish(*path);
    } else {
        RCLCPP_INFO(this->get_logger(), "Backend failed to produce path.");
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
