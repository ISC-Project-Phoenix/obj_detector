#pragma once

#include "obj_detector/IDetectionStrategy.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/node.hpp"

/// Detector for traffic cones, using deterministic CV techniques.
class ConeDetectionStrategy : public IDetectionStrategy {
    const rclcpp::Node& parent;

public:
    ConeDetectionStrategy(const rclcpp::Node& parent) : parent(parent){};

    /// Detects the center points of orange traffic cones.
    std::vector<cv::Point2d> detect_objects(const cv::Mat& rgb_mat) override;

    virtual ~ConeDetectionStrategy() = default;
};
