#pragma once

#include "IFrontEnd.hpp"
#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"

struct ConvexMethodParams {
    /// Angle around 90 degrees to consider straight
    float turn_threshold{20.0};               // meters
    double end_segment_angle_threshold{2.2};  // radians
    double cluster_threshold{2.0};            // meters
    /// Smallest hull considered containing both sides
    double convex_hull_area_threshold{13.0};
    bool debug{false};
};

struct TurningScenarioParams {
    double end_segment_angle_threshold;  // radians
    double cluster_threshold;            // meters
};

enum class Scenario : std::uint8_t { kStraight = 0, kLeft = 1U, kRight = 2U };

/// Classifies detections in some state
class IScenarioClassifier {
public:
    /// Classifies detections into right and left sides of the track. If only one side is visible, return none.
    virtual LeftRightResults classify(std::vector<cv::Point2d>& convex_hull,
                                      const std::vector<cv::Point2d>& detections_2d) = 0;
    virtual ~IScenarioClassifier() = default;
};

class TurningScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(std::vector<cv::Point2d>& convex_hull,
                              const std::vector<cv::Point2d>& detections_2d) override;

protected:
    explicit TurningScenarioClassifier(const TurningScenarioParams& params) : params{params} {}

    virtual std::size_t find_start_idx_on_convex_hull(const std::vector<double>& convex_hull_angles) = 0;
    virtual std::vector<cv::Point2d>& get_outside_detections_result(LeftRightResults& classification) = 0;
    virtual std::vector<cv::Point2d>& get_inside_detections_result(LeftRightResults& classification) = 0;

private:
    /// Find the angle between segments along the convex hull. If the angle is less
    /// than end_segment_angle_threshold, then we found the end segment.
    std::size_t find_end_segment_index(const std::vector<cv::Point2d>& convex_hull);

    /// Find the detection with the max euclidean distance relative to [0,0].
    std::size_t find_idx_of_max_distance(const std::vector<cv::Point2d>& convex_hull);

    /// Find the detections that have not been classified yet.
    std::vector<cv::Point2d> find_unused_detections(const std::vector<cv::Point2d>& all_detections,
                                                    const std::vector<cv::Point2d>& used_detections);

    /// Associate unused detections thus far with what has been classified. This is needed
    /// since the convex hull may not necessarily contain all detections on the outside
    /// of the turn.
    void associate_unused_detections(std::vector<cv::Point2d>& used_detections,
                                     const std::vector<cv::Point2d>& unused_detections);

    TurningScenarioParams params;
};

class LeftScenarioClassifier : public TurningScenarioClassifier {
public:
    explicit LeftScenarioClassifier(const TurningScenarioParams& params) : TurningScenarioClassifier(params) {}

    LeftRightResults classify(std::vector<cv::Point2d>& convex_hull,
                              const std::vector<cv::Point2d>& detections_2d) override;

protected:
    std::size_t find_start_idx_on_convex_hull(const std::vector<double>& convex_hull_angles) override;

    std::vector<cv::Point2d>& get_outside_detections_result(LeftRightResults& classification) override;

    std::vector<cv::Point2d>& get_inside_detections_result(LeftRightResults& classification) override;
};

class RightScenarioClassifier : public TurningScenarioClassifier {
public:
    explicit RightScenarioClassifier(const TurningScenarioParams& params) : TurningScenarioClassifier(params) {}

    LeftRightResults classify(std::vector<cv::Point2d>& convex_hull,
                              const std::vector<cv::Point2d>& detections_2d) override;

protected:
    std::size_t find_start_idx_on_convex_hull(const std::vector<double>& convex_hull_angles) override;

    std::vector<cv::Point2d>& get_outside_detections_result(LeftRightResults& classification) override;

    std::vector<cv::Point2d>& get_inside_detections_result(LeftRightResults& classification) override;
};

class StraightScenarioClassifier : public IScenarioClassifier {
public:
    LeftRightResults classify(std::vector<cv::Point2d>& convex_hull,
                              const std::vector<cv::Point2d>& detections_2d) override;
};

/// ConvexMethod performs left right classification on an array of 2d points
class ConvexMethod : public IFrontEnd {
public:
    explicit ConvexMethod(const ConvexMethodParams& params, std::optional<rclcpp::Logger> logger = std::nullopt)
        : logger{std::move(logger)}, params{params} {
        const TurningScenarioParams scenario_params{params.end_segment_angle_threshold, params.cluster_threshold};

        left_scenario_classifier = std::make_shared<LeftScenarioClassifier>(scenario_params);
        right_scenario_classifier = std::make_shared<RightScenarioClassifier>(scenario_params);
        straight_scenario_classifier = std::make_shared<StraightScenarioClassifier>();
    }

    /// Perform left right classification
    std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& detections_array) override;

private:
    /// Set detections_2d
    std::vector<cv::Point2d> get_detections_vector(const geometry_msgs::msg::PoseArray& detections_array);

    /// Calculate and return the counter clock-wise convex hull of a vector of 2d points
    std::vector<cv::Point2d> get_convex_hull(const std::vector<cv::Point2d>& detections_2d);

    /// Check area of convex hull to determine if using this algo is valid
    bool is_convex_hull_valid(const std::vector<cv::Point2d>& convex_hull);

    /// Determine if detections indicate we are turning left, right, or staying straight
    std::tuple<Scenario, cv::Vec2f> determine_scenario(const std::vector<cv::Point2d>& detections_2d);

    /// Removes cones that are too close to each other, if the detector bugs out
    std::vector<cv::Point2d> remove_duplicate_detections(const std::vector<cv::Point2d>& detections_2d);

    /// Visualizes the classifications and hull in an opencv window
    static void visualize_hull(std::optional<LeftRightResults>& classification, std::vector<cv::Point2d>& convex_hull,
                               Scenario scenario, const cv::Vec2f& line);

    std::optional<rclcpp::Logger> logger;

    ConvexMethodParams params{};

    // These are our possible states
    std::shared_ptr<LeftScenarioClassifier> left_scenario_classifier;
    std::shared_ptr<RightScenarioClassifier> right_scenario_classifier;
    std::shared_ptr<StraightScenarioClassifier> straight_scenario_classifier;
    // This will be copied over by one of the above states for each cycle, then dispatched
    std::shared_ptr<IScenarioClassifier> scenario_classifier;
};
