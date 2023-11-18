#pragma once

#include "IFrontEnd.hpp"

class ConvexMethod : public IFontEnd {
public:
    std::optional<LeftRightResults> classify(geometry_msgs::msg::PoseArray);
};