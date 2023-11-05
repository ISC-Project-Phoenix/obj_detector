#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "obj_detector/ObjDetectorNode_node.hpp"

TEST(ObjDetectorNode, Test1) {}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}