#pragma once

#include <optional>

#include "misc.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class IFontEnd {
public:
    virtual std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& detections_array) = 0;
    virtual ~IFontEnd() = default;
};
