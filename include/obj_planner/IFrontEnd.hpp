#pragma once

#include <optional>

#include "geometry_msgs/msg/pose_array.hpp"
#include "misc.hpp"

/// Classifies detections from across the road into left and right categories.
class IFrontEnd {
public:
    /// Perform left right classification. Returns none if there is only one side.
    virtual std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& detections_array) = 0;
    virtual ~IFrontEnd() = default;
};
