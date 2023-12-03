#pragma once

#include <optional>

#include "ObjPlannerNode_node.hpp"
#include "nav_msgs/msg/path.hpp"

class IBackEnd {
    /// Takes classified cones and produces a path to publish
    virtual std::optional<nav_msgs::msg::Path> create_path(const LeftRightResults& cones) = 0;
};
