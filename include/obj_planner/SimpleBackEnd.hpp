#pragma once

#include "IBackEnd.hpp"

class SimpleBackEnd : public IBackEnd {
public:
    std::optional<nav_msgs::msg::Path> create_path(LeftRightResults);
};
