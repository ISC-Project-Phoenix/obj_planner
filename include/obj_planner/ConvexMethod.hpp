#pragma once

#include "IFrontEnd.hpp"
#include "opencv2/core/types.hpp"

struct ConvexMethodParams {
    float turn_threshold{1.0F};               // meters
    float end_segment_angle_threshold{2.2F};  // radians
    float cluster_threshold{2.0F};            // meters
};

enum class Scenario : std::uint8_t { kStraight = 0, kLeft = 1U, kRight = 2U };

class IScenarioClassifier {
public:
    virtual LeftRightResults classify(const std::vector<cv::Point>& convex_hull,
                                      const std::vector<cv::Point>& cones_2d) = 0;
    virtual ~IScenarioClassifier() = default;
};

class LeftScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(const std::vector<cv::Point>& convex_hull,
                              const std::vector<cv::Point>& cones_2d) override;
};

class RightScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(const std::vector<cv::Point>& convex_hull,
                              const std::vector<cv::Point>& cones_2d) override;
};

class StraightScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(const std::vector<cv::Point>& convex_hull,
                              const std::vector<cv::Point>& cones_2d) override;
};

// ConvexMethod performs left right classification on an array of 2d points
class ConvexMethod : public IFontEnd {
public:
    explicit ConvexMethod(const ConvexMethodParams& params)
        : params{params},
          left_scenario_classifier{std::make_shared<LeftScenarioClassifier>()},
          right_scenario_classifier{std::make_shared<RightScenarioClassifier>()},
          straight_scenario_classifier{std::make_shared<StraightScenarioClassifier>()} {}

    // Perform left right classification
    std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& cones_array) override;

private:
    // Set cones_2d
    std::vector<cv::Point> get_cones_vector(const geometry_msgs::msg::PoseArray& cones_array);

    // Calculate and return the convex hull of a vector of 2d points
    std::vector<cv::Point> get_convex_hull();

    // Check area of convex hull to determine if using this algo is valid
    bool is_convex_hull_valid();

    // Determine if cones indicate we are turning left, right, or staying straight
    Scenario determine_scenario();

    ConvexMethodParams params{};
    std::shared_ptr<LeftScenarioClassifier> left_scenario_classifier;
    std::shared_ptr<RightScenarioClassifier> right_scenario_classifier;
    std::shared_ptr<StraightScenarioClassifier> straight_scenario_classifier;
    std::shared_ptr<IScenarioClassifier> scenario_classifier;
    std::vector<cv::Point> cones_2d{};
};
