#pragma once

#include "IBackEnd.hpp"

struct SimpleBackEndParams {
    /// max distance allowed between cones in meters
    float tolerance_value{20.0};
};

/// Naively pairs detections across the track, creating a path of midpoints
class SimpleBackEnd : public IBackEnd {
public:
    explicit SimpleBackEnd(const SimpleBackEndParams& params) : params{params} {}

    std::optional<nav_msgs::msg::Path> create_path(const LeftRightResults& detections, std::string_view frame) override;

private:
    SimpleBackEndParams params;
};