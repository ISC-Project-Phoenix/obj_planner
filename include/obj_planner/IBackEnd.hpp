#pragma once

#include <optional>

#include "misc.hpp"
#include "nav_msgs/msg/path.hpp"

class IBackEnd {
public:
    /// Takes classified detections and produces a path to publish
    virtual std::optional<nav_msgs::msg::Path> create_path(const LeftRightResults& detections) = 0;
    virtual ~IBackEnd() = default;
};
