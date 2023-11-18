#pragma once

#include <optional>

#include "geometry_msgs/msg/pose_array.hpp"
#include "opencv2/core/types.hpp"

struct LeftRightResults {
    std::vector<cv::Point2d> left_cones;
    std::vector<cv::Point2d> right_cones;
};

class IFontEnd {
    virtual std::optional<LeftRightResults> classify(geometry_msgs::msg::PoseArray) = 0;
};
