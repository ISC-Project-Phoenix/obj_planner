#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "obj_planner/ConvexMethod.hpp"
#include "obj_planner/ObjPlannerNode_node.hpp"

bool are_detection_vectors_equal(const std::vector<cv::Point2d>& lhs, const std::vector<cv::Point2d>& rhs) {
    bool result{true};
    constexpr double epsilon{1e-1};
    if (lhs.size() == rhs.size()) {
        for (std::size_t i{0U}; i < lhs.size(); ++i) {
            if (std::abs(lhs[i].x - rhs[i].x) > epsilon || std::abs(lhs[i].y - rhs[i].y) > epsilon) {
                result = false;
                break;
            }
        }
    } else {
        result = false;
    }

    return result;
}

class ConvexMethodTestFixture : public testing::Test {
protected:
    ConvexMethodTestFixture() : params{}, unit{params} {}
    ~ConvexMethodTestFixture() {}

    ConvexMethodParams params{};
    ConvexMethod unit;
};

TEST_F(ConvexMethodTestFixture, GivenNoInput_ExpectNullOpt) {
    const auto classification = unit.classify(geometry_msgs::msg::PoseArray{});
    EXPECT_FALSE(classification.has_value());
}

TEST_F(ConvexMethodTestFixture, GivenInputWithSmallArea_ExpectNullOpt) {
    geometry_msgs::msg::PoseArray input{};

    std::vector<cv::Point2d> points{};
    points.push_back(cv::Point2d{2.0, -2.0});
    points.push_back(cv::Point2d{4.0, -2.0});
    points.push_back(cv::Point2d{2.0, 2.0});
    points.push_back(cv::Point2d{4.0, 2.0});

    std::transform(std::cbegin(points), std::cend(points), std::back_inserter(input.poses), [](const auto& point) {
        geometry_msgs::msg::Pose pose{};
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0.0;
        return pose;
    });

    const auto classification = unit.classify(input);
    EXPECT_FALSE(classification.has_value());
}

struct ConvexMethodTestParam {
    std::string name;
    geometry_msgs::msg::PoseArray input;
    std::optional<LeftRightResults> expected;
};

class ConvexMethodParameterizedFixture : public ConvexMethodTestFixture,
                                         public testing::WithParamInterface<ConvexMethodTestParam> {};

template <class TestParam>
std::string print_test_name(const testing::TestParamInfo<TestParam>& info) {
    return info.param.name;
}

TEST_P(ConvexMethodParameterizedFixture, GivenInput_ExpectCorrectResults) {
    const auto param = GetParam();
    const auto actual = unit.classify(param.input);
    ASSERT_TRUE(actual.has_value() == param.expected.has_value());

    if (param.expected.has_value()) {
        const auto& exp_detections = param.expected.value();
        const auto& act_detections = actual.value();
        EXPECT_TRUE(are_detection_vectors_equal(act_detections.left_detections, exp_detections.left_detections));
        EXPECT_TRUE(are_detection_vectors_equal(act_detections.right_detections, exp_detections.right_detections));
    }
}

// Straight road nominal scenario. Algo should classify everything properly
geometry_msgs::msg::PoseArray generate_straight_scenario() {
    geometry_msgs::msg::PoseArray input{};

    std::vector<cv::Point2d> points{};
    points.push_back(cv::Point2d{0.8, -3.0});
    points.push_back(cv::Point2d{0.9, 3.0});
    points.push_back(cv::Point2d{3.0, -3.0});
    points.push_back(cv::Point2d{3.6, 3.0});
    points.push_back(cv::Point2d{7.1, -3.0});
    points.push_back(cv::Point2d{7.3, 3.0});
    points.push_back(cv::Point2d{9.9, -3.0});
    points.push_back(cv::Point2d{9.1, 3.0});

    std::transform(std::cbegin(points), std::cend(points), std::back_inserter(input.poses), [](const auto& point) {
        geometry_msgs::msg::Pose pose{};
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0.0;
        return pose;
    });

    return input;
}

// Tight left turn scenario of 6.5m radius nominal scenario. Algo should classify everything properly.
// This scneario is generated and stored in "cone_coords_6.5m_turn_5.csv". The corresponding results are
// generated via matlab implementation.
geometry_msgs::msg::PoseArray generate_left_6_5m_turn_scenario() {
    geometry_msgs::msg::PoseArray input{};

    std::vector<cv::Point2d> points{};
    points.push_back(cv::Point2d{0.75, 2.61});
    points.push_back(cv::Point2d{0.76, -2.65});
    points.push_back(cv::Point2d{2.20, 3.03});
    points.push_back(cv::Point2d{4.43, -1.81});
    points.push_back(cv::Point2d{3.04, 3.65});
    points.push_back(cv::Point2d{7.83, 0.08});
    points.push_back(cv::Point2d{4.60, 5.08});
    points.push_back(cv::Point2d{10.10, 3.13});
    points.push_back(cv::Point2d{4.14, 6.49});
    points.push_back(cv::Point2d{10.92, 6.33});
    points.push_back(cv::Point2d{4.92, 7.94});
    points.push_back(cv::Point2d{9.44, 10.16});

    std::transform(std::cbegin(points), std::cend(points), std::back_inserter(input.poses), [](const auto& point) {
        geometry_msgs::msg::Pose pose{};
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0.0;
        return pose;
    });

    return input;
}

// Tight right turn scenario of 6.5m radius nominal scenario. Algo should classify everything properly.
// This scneario is generated and stored in "cone_coords_6.5m_right_turn_1.csv". The corresponding results are
// generated via matlab implementation.
geometry_msgs::msg::PoseArray generate_right_6_5m_turn_scenario() {
    geometry_msgs::msg::PoseArray input{};

    std::vector<cv::Point2d> points{};
    points.push_back(cv::Point2d{0.36, -2.59});
    points.push_back(cv::Point2d{0.50, 2.57});
    points.push_back(cv::Point2d{2.40, -3.02});
    points.push_back(cv::Point2d{5.02, 1.69});
    points.push_back(cv::Point2d{3.87, -3.87});
    points.push_back(cv::Point2d{7.40, -0.48});
    points.push_back(cv::Point2d{3.69, -5.30});
    points.push_back(cv::Point2d{8.48, -3.64});
    points.push_back(cv::Point2d{3.98, -6.78});
    points.push_back(cv::Point2d{10.42, -6.96});
    points.push_back(cv::Point2d{3.47, -8.32});
    points.push_back(cv::Point2d{9.43, -10.98});

    std::transform(std::cbegin(points), std::cend(points), std::back_inserter(input.poses), [](const auto& point) {
        geometry_msgs::msg::Pose pose{};
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0.0;
        return pose;
    });

    return input;
}

// Tight left turn scenario of 14m radius NOT nominal scenario. Algo makes a mistake in this data set.
// This scneario is generated and stored in "cone_coords_14.0m_turn_9.csv". The corresponding results are
// generated via matlab implementation.
geometry_msgs::msg::PoseArray generate_left_14m_turn_scenario() {
    geometry_msgs::msg::PoseArray input{};

    std::vector<cv::Point2d> points{};
    points.push_back(cv::Point2d{0.58, 2.63});
    points.push_back(cv::Point2d{0.96, -2.61});
    points.push_back(cv::Point2d{3.45, 2.85});
    points.push_back(cv::Point2d{4.10, -2.20});
    points.push_back(cv::Point2d{5.07, 3.69});
    points.push_back(cv::Point2d{7.29, -1.18});
    points.push_back(cv::Point2d{7.68, 4.54});
    points.push_back(cv::Point2d{10.70, 0.19});

    std::transform(std::cbegin(points), std::cend(points), std::back_inserter(input.poses), [](const auto& point) {
        geometry_msgs::msg::Pose pose{};
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0.0;
        return pose;
    });

    return input;
}

INSTANTIATE_TEST_SUITE_P(
    ConvexMethodPerformanceTests, ConvexMethodParameterizedFixture,
    testing::Values(
        ConvexMethodTestParam{"StraightRoadScenario", generate_straight_scenario(),
                              LeftRightResults{{{0.9, 3.0}, {3.6, 3.0}, {7.3, 3.0}, {9.1, 3.0}},
                                               {{0.8, -3.0}, {3.0, -3.0}, {7.1, -3.0}, {9.9, -3.0}}}},
        ConvexMethodTestParam{
            "LeftTightTurnScenario", generate_left_6_5m_turn_scenario(),
            LeftRightResults{
                {{0.75, 2.61}, {2.20, 3.03}, {3.04, 3.65}, {4.60, 5.08}, {4.14, 6.49}, {4.92, 7.94}},
                {{0.76, -2.65}, {4.43, -1.81}, {7.83, 0.08}, {10.10, 3.13}, {10.92, 6.33}, {9.44, 10.16}}}},
        ConvexMethodTestParam{
            "RightTightTurnScenario", generate_right_6_5m_turn_scenario(),
            LeftRightResults{
                {{0.50, 2.57}, {5.02, 1.69}, {7.40, -0.48}, {10.42, -6.96}, {9.43, -10.98}, {8.48, -3.64}},
                {{0.36, -2.59}, {2.40, -3.02}, {3.87, -3.87}, {3.69, -5.30}, {3.98, -6.78}, {3.47, -8.32}}}},
        ConvexMethodTestParam{"LeftTurnWithIssueScenario", generate_left_14m_turn_scenario(),
                              LeftRightResults{{{0.58, 2.63}, {3.45, 2.85}, {5.07, 3.69}, {7.68, 4.54}, {10.70, 0.19}},
                                               {{0.96, -2.61}, {4.10, -2.20}, {7.29, -1.18}}}}),
    print_test_name<ConvexMethodTestParam>);

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}