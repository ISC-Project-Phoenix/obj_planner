#pragma once

#include "IFrontEnd.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class ConvexMethod : public IFontEnd {

    public:
        void pushConeCoord();
        std::optional<LeftRightResults> classify(geometry_msgs::msg::PoseArray);
};