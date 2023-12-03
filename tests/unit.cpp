#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "obj_planner/ConvexMethod.hpp"
#include "obj_planner/ObjPlannerNode_node.hpp"

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

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}