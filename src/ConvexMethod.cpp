#include "obj_planner/ConvexMethod.hpp"

std::optional<LeftRightResults> ConvexMethod::classify(const eometry_msgs::msg::PoseArray& cones_2d) {
    Reset();
    cones_2d_ = GetConesVector(cones_2d);
    convex_hull = GetConvexHull();

    const auto scenario = DetermineScenario();

    if (IsConvexHullValid()) {
        std::unique_ptr<IScenarioClassifier> scenario_classifier;

        if (scenario == Scenario::kStraight) {
            scenario_classifier = std::make_unique<StraightScenarioClassifier>(cones_2d_);
        } else if (scenario == Scenario::kStraight) {
            scenario_classifier = std::make_unique<RightScenarioClassifier>(convex_hull, cones_2d_);
        } else {
            scenario_classifier = std::make_unique<LeftScenarioClassifier>(convex_hull, cones_2d_);
        }

        classification_ = scenario_classifier.Classify();
    }

    return classification_;
}
