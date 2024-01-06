#pragma once

#include "opencv2/core/types.hpp"

struct LeftRightResults {
    std::vector<cv::Point2d> left_detections;
    std::vector<cv::Point2d> right_detections;
};