#include "obj_planner/ConvexMethod.hpp"

#include <numeric>

#include "opencv2/imgproc.hpp"

std::optional<LeftRightResults> ConvexMethod::classify(const geometry_msgs::msg::PoseArray& detections_array) {
    std::optional<LeftRightResults> classification{std::nullopt};
    const auto detections_2d = get_detections_vector(detections_array);
    const auto convex_hull = get_convex_hull(detections_2d);

    if (is_convex_hull_valid()) {
        const auto scenario = determine_scenario(detections_2d);

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

std::vector<cv::Point2d> ConvexMethod::get_detections_vector(const geometry_msgs::msg::PoseArray& detections_array) {
    std::vector<cv::Point2d> detections_2d{};
    for (const auto& pose : detections_array.poses) {
        detections_2d.emplace_back(pose.position.x, pose.position.y);
    }
    return detections_2d;
}

std::vector<cv::Point2d> ConvexMethod::get_convex_hull(const std::vector<cv::Point2d>& detections_2d) {
    std::vector<cv::Point2d> convex_hull{};
    cv::convexHull(detections_2d, convex_hull, false);
    return convex_hull;
}

bool ConvexMethod::is_convex_hull_valid() {
    //TODO For now set to true, add some area based impl.
    return true;
}

Scenario ConvexMethod::determine_scenario(const std::vector<cv::Point2d>& detections_2d) {
    //TODO impl
    Scenario scenario{Scenario::kStraight};

    if (detections_2d.size() > 0) {
        const double sum_in_y =
            std::accumulate(std::begin(detections_2d), std::end(detections_2d), 0.,
                            [](const auto& accumulation, const auto& point) { return accumulation + point.y; });
        const double center_of_mass_in_y{sum_in_y / detections_2d.size()};
        if (center_of_mass_in_y > params.turn_threshold) {
            scenario = Scenario::kLeft;
        } else if (center_of_mass_in_y < -params.turn_threshold) {
            scenario = Scenario::kRight;
        }
    }
    return scenario;
}

LeftRightResults StraightScenarioClassifier::classify(const std::vector<cv::Point2d>& convex_hull,
                                                      const std::vector<cv::Point2d>& detections_2d) {
    //TODO impl
    (void)convex_hull;
    (void)detections_2d;
    return {};
}

LeftRightResults LeftScenarioClassifier::classify(const std::vector<cv::Point2d>& convex_hull,
                                                  const std::vector<cv::Point2d>& detections_2d) {
    //TODO impl
    (void)convex_hull;
    (void)detections_2d;
    return {};
}

LeftRightResults RightScenarioClassifier::classify(const std::vector<cv::Point2d>& convex_hull,
                                                   const std::vector<cv::Point2d>& detections_2d) {
    //TODO impl
    (void)convex_hull;
    (void)detections_2d;
    return {};
}
