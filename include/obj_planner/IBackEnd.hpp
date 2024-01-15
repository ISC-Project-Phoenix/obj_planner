#pragma once

#include <optional>

#include "misc.hpp"
#include "nav_msgs/msg/path.hpp"

/// Converts classified objects into a path.
class IBackEnd {
public:
    /// Takes classified detections and produces a path to publish.
    ///
    /// \param detections Classified objects from the frontend
    /// \param frame The frame we are operating in
    /// \return A path, if one is possible to create.
    virtual std::optional<nav_msgs::msg::Path> create_path(const LeftRightResults& detections,
                                                           std::string_view frame) = 0;
    virtual ~IBackEnd() = default;
};
