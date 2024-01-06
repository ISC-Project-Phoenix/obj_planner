#pragma once

#include <optional>

#include "geometry_msgs/msg/pose_array.hpp"
#include "misc.hpp"

class IFontEnd {
public:
    virtual std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& detections_array) = 0;
    virtual ~IFontEnd() = default;
};
