#pragma once

#include <optional>

#include "geometry_msgs/msg/pose_array.hpp"
#include "misc.hpp"

class IFrontEnd {
public:
    /// Perform left right classification. Returns none if there is only one side.
    virtual std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& detections_array) = 0;
    virtual ~IFrontEnd() = default;
};
