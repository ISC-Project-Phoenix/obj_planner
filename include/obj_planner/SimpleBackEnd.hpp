#pragma once

#include "IBackEnd.hpp"

/// Naively pairs detections across the track, creating a path of midpoints
struct SimpleBackEndParams {
    /// max distance allowed between cones in meters
    float tolerance_value{20.0};  // Param
};

/// Naively pairs detections across the track, creating a path of midpoints
class SimpleBackEnd : public IBackEnd {
public:
    //create constructora
    explicit SimpleBackEnd(const SimpleBackEndParams& params) : params{params} {}

    std::optional<nav_msgs::msg::Path> create_path(const LeftRightResults& detections) override;

private:
    SimpleBackEndParams params;
};