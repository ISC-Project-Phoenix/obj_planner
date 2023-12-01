#pragma once

#include <optional>

#include "ObjPlannerNode_node.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class IFontEnd {
    virtual std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray&) = 0;
};
