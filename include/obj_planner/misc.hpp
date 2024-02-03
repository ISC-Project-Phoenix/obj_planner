#pragma once

#include "opencv2/core/types.hpp"

/// Classification results. Each side will always have at least one detection. These are not garrentied to be sorted.
struct LeftRightResults {
    std::vector<cv::Point2d> left_detections;
    std::vector<cv::Point2d> right_detections;
};