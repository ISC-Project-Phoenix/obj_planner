#include "obj_planner/SimpleBackEnd.hpp"

std::optional<nav_msgs::msg::Path> SimpleBackEnd::create_path(const LeftRightResults& detections) {
    std::optional<nav_msgs::msg::Path> Path_Point{std::nullopt};

    std::vector<cv::Point2d> path;
    //Sets vec_more and vec_less to proper side
    auto& vec_more = (detections.left_detections.size() <= detections.right_detections.size())
                         ? detections.right_detections
                         : detections.left_detections;
    auto& vec_less = (detections.left_detections.size() > detections.right_detections.size())
                         ? detections.left_detections
                         : detections.right_detections;

    //params.tolerance_value
    for (auto& more_point : vec_more) {
        std::vector<double> distance;
        cv::Point2d nearest;

        for (auto& less_point : vec_less) {
            //distances between cone[more_point] on longer and cones on shorter
            distance.push_back(std::hypot(more_point.x - less_point.x, more_point.y - less_point.y));
        }
        //finds index of vec_less where the distance is shortest
        auto min_value = *std::min_element(distance.begin(), distance.end());
        if (min_value <= params.tolerance_value) {  //suggestion from nate

            auto index = std::find(distance.begin(), distance.end(), min_value);
            nearest = vec_less[std::distance(distance.begin(), index)];

            //Midpoint form of cone on longer to nearest cone on shorter
            cv::Point2d temp;
            temp.x = (more_point.x + nearest.x) / 2;
            temp.y = (more_point.y + nearest.y) / 2;
            path.push_back(temp);
            //MUST ADD PATH_POINTS TO PATH AND RETURN
        }
    }
    return Path_Point;
}
