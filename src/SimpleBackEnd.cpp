#include "obj_planner/SimpleBackEnd.hpp"

std::optional<nav_msgs::msg::Path> SimpleBackEnd::create_path(const LeftRightResults& detections) {
    return std::nullopt;
}
//TODO    Given 2 vectors left_detections and right_detections
// 1. Choose side with more cones
// 2. For cone in longer
//      - Find nearest cone on shorter
//      - if no nearest
//          -halt
//      - if nearest >= tolerence
//          - skip cone
//      - path point = midpoint(nearest, cone)
//      -path.apend(path point) // put in vector
//          return path