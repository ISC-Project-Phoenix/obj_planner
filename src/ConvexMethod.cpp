#include "obj_planner/ConvexMethod.hpp"

std::optional<LeftRightResults> ConvexMethod::classify(const geometry_msgs::msg::PoseArray& detections_array) {
    std::optional<LeftRightResults> classification{std::nullopt};
    auto detections_2d = get_detections_vector(detections_array);
    const auto convex_hull = get_convex_hull();

    if (is_convex_hull_valid()) {
        const auto scenario = determine_scenario();

        if (scenario == Scenario::kStraight) {
            scenario_classifier = straight_scenario_classifier;
        } else if (scenario == Scenario::kLeft) {
            scenario_classifier = left_scenario_classifier;
        } else {
            scenario_classifier = right_scenario_classifier;
        }

        classification = scenario_classifier->classify(convex_hull, detections_2d);
    }

    return classification;
}

std::vector<cv::Point> ConvexMethod::get_detections_vector(const geometry_msgs::msg::PoseArray& detections_2d) {
    //TODO impl
    (void)detections_2d;
    return {};
}

std::vector<cv::Point> ConvexMethod::get_convex_hull() {
    //TODO impl
    return {};
}

bool ConvexMethod::is_convex_hull_valid() {
    //TODO impl
    return false;
}

Scenario ConvexMethod::determine_scenario() {
    //TODO impl
    return Scenario::kStraight;
}

LeftRightResults StraightScenarioClassifier::classify(const std::vector<cv::Point>& convex_hull,
                                                      const std::vector<cv::Point>& detections_2d) {
    //TODO impl
    (void)convex_hull;
    (void)detections_2d;
    return {};
}

LeftRightResults LeftScenarioClassifier::classify(const std::vector<cv::Point>& convex_hull,
                                                  const std::vector<cv::Point>& detections_2d) {
    //TODO impl
    (void)convex_hull;
    (void)detections_2d;
    return {};
}

LeftRightResults RightScenarioClassifier::classify(const std::vector<cv::Point>& convex_hull,
                                                   const std::vector<cv::Point>& detections_2d) {
    //TODO impl
    (void)convex_hull;
    (void)detections_2d;
    return {};
}
