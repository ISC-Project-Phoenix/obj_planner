#pragma once

#include "IFrontEnd.hpp"
#include "opencv2/core/types.hpp"

struct ConvexMethodParams {
    double turn_threshold{1.0F};              // meters
    float end_segment_angle_threshold{2.2F};  // radians
    float cluster_threshold{2.0F};            // meters
};

enum class Scenario : std::uint8_t { kStraight = 0, kLeft = 1U, kRight = 2U };

/// Classifies detections in some state
class IScenarioClassifier {
public:
    /// Classifies detections into right and left sides of the track. If only one side is visible, return none.
    virtual LeftRightResults classify(const std::vector<cv::Point2d>& convex_hull,
                                      const std::vector<cv::Point2d>& detections_2d) = 0;
    virtual ~IScenarioClassifier() = default;
};

class LeftScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(const std::vector<cv::Point2d>& convex_hull,
                              const std::vector<cv::Point2d>& detections_2d) override;
};

class RightScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(const std::vector<cv::Point2d>& convex_hull,
                              const std::vector<cv::Point2d>& detections_2d) override;
};

class StraightScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(const std::vector<cv::Point2d>& convex_hull,
                              const std::vector<cv::Point2d>& detections_2d) override;
};

/// ConvexMethod performs left right classification on an array of 2d points
class ConvexMethod : public IFontEnd {
public:
    explicit ConvexMethod(const ConvexMethodParams& params)
        : params{params},
          left_scenario_classifier{std::make_shared<LeftScenarioClassifier>()},
          right_scenario_classifier{std::make_shared<RightScenarioClassifier>()},
          straight_scenario_classifier{std::make_shared<StraightScenarioClassifier>()} {}

    /// Perform left right classification
    std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& detections_array) override;

private:
    /// Set detections_2d
    std::vector<cv::Point2d> get_detections_vector(const geometry_msgs::msg::PoseArray& detections_array);

    /// Calculate and return the counter clock-wise convex hull of a vector of 2d points
    std::vector<cv::Point2d> get_convex_hull(const std::vector<cv::Point2d>& detections_2d);

    /// Check area of convex hull to determine if using this algo is valid
    bool is_convex_hull_valid();

    /// Determine if detections indicate we are turning left, right, or staying straight
    Scenario determine_scenario(const std::vector<cv::Point2d>& detections_2d);

    ConvexMethodParams params{};

    // These are our possible states
    std::shared_ptr<LeftScenarioClassifier> left_scenario_classifier;
    std::shared_ptr<RightScenarioClassifier> right_scenario_classifier;
    std::shared_ptr<StraightScenarioClassifier> straight_scenario_classifier;
    // This will be copied over by one of the above states for each cycle, then dispatched
    std::shared_ptr<IScenarioClassifier> scenario_classifier;
};
