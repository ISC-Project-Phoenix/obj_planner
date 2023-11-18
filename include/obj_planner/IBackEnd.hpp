#pragma once

#include <optional>

#include "ObjPlannerNode_node.hpp"
#include "nav_msgs/msg/path.hpp"

class IBackEnd {
    virtual std::optional<nav_msgs::msg::Path> create_path(LeftRightResults) = 0;
};
