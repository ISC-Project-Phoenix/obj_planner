#pragma once

#include "IFrontEnd.hpp"
#include "opencv2/core/types.hpp"

struct ConvexMethodParams {
    float turn_threshold{1.0F};               // meters
    float end_segment_angle_threshold{2.2F};  // radians
    float cluster_threshold{2.0F};            // meters
};

enum class Scenario : std::uint8_t { kStraight = 0, kLeft = 1U, kRight = 2U };

// ConvexMethod performs left right classification on an array of 2d points
class ConvexMethod : public IFontEnd {
public:
    explicit ConvexMethod(const ConvexMethodParams& params) : params_{params} {}

    // Perform left right classification and set private member `cones_2d`
    std::optional<LeftRightResults> classify(const geometry_msgs::msg::PoseArray& cones_2d);

private:
    // Reset class members
    void Reset();

    // Set cones_2d
    std::vector<cv::Point> GetConesVector(const geometry_msgs::msg::PoseArray& cones_2d);

    // Calculate and return the convex hull of a vector of 2d points
    std::vector<cv::Point> GetConvexHull();

    // Check area of convex hull to determine if using this algo is valid
    bool IsConvexHullValid();

    // Determine if cones indicate we are turning left, right, or staying straight
    Scenario DetermineScenario();

    ConvexMethodParams params_{};
    std::optional<LeftRightResults> classification_{std::nullopt};
    std::vector<cv::Point> cones_2d_{};
};

class IScenarioClassifier {
public:
    virtual LeftRightResults Classify() = 0;
    virtual ~IScenarioClassifier() = default;
};

class LeftScenarioClassifier : public IScenarioClassifier {
public:
    void LeftScenarioClassifier(const std::vector<cv::Point>& convex_hull, const std::vector<cv::Point>& cones_2d)
        : convex_hull_{convex_hull}, cones_2d_{cones_2d} {}

    LeftRightResults Classify();

private:
    std::vector<cv::Point> convex_hull_{};
    std::vector<cv::Point> cones_2d_{};
    LeftRightResults classification_{};
};

class RightScenarioClassifier : public IScenarioClassifier {
public:
    void RightScenarioClassifier(const std::vector<cv::Point>& convex_hull, const std::vector<cv::Point>& cones_2d)
        : convex_hull_{convex_hull}, cones_2d_{cones_2d} {}

    LeftRightResults Classify();

private:
    std::vector<cv::Point> convex_hull_{};
    std::vector<cv::Point> cones_2d_{};
    LeftRightResults classification_{};
};

class StraightScenarioClassifier : public IScenarioClassifier {
public:
    void LeftScenarioClassifier(const std::vector<cv::Point>& cones_2d) : cones_2d_{cones_2d} {}

    LeftRightResults Classify();

private:
    std::vector<cv::Point> cones_2d_{};
    LeftRightResults classification_{};
};
