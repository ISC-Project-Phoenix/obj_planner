#pragma once

#include "IBackEnd.hpp"

/// Naively pairs cones across the track, creating a path of midpoints
class SimpleBackEnd : public IBackEnd {
public:
    std::optional<nav_msgs::msg::Path> create_path(const LeftRightResults& cones) override;
};
