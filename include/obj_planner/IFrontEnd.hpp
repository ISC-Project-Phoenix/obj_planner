#pragma once

#include <optional>

#include "ObjPlannerNode_node.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class IFontEnd {
public:
    virtual std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& cones_array) = 0;
    virtual ~IFontEnd() = default;
};
