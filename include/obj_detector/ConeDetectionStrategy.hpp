#pragma once

#include "obj_detector/IDetectionStrategy.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/node.hpp"

/// Detector for traffic cones, using deterministic CV techniques.
class ConeDetectionStrategy : public IDetectionStrategy {
    const rclcpp::Node& parent;
    double area_threshold;
    double aspect_ratio_threshold_min;
    double aspect_ratio_threshold_max;
    double area_perimeter_ratio;
    bool debug;

    cv::Scalar upperb;
    cv::Scalar lowerb;
    float gamma;

public:
    ConeDetectionStrategy(const rclcpp::Node& parent) : parent(parent) {
        // Copy params from node
        area_threshold = parent.get_parameter("area_threshold").as_double();
        aspect_ratio_threshold_min = parent.get_parameter("aspect_ratio_threshold_min").as_double();
        aspect_ratio_threshold_max = parent.get_parameter("aspect_ratio_threshold_max").as_double();
        area_perimeter_ratio = parent.get_parameter("area_perimeter_ratio").as_double();
        debug = parent.get_parameter("debug").as_bool();
        gamma = float(parent.get_parameter("gamma").as_double());

        // Parse hsv bounds from arrays
        auto upperb_raw = parent.get_parameter("hsv_upperb").as_integer_array();
        auto lowerb_raw = parent.get_parameter("hsv_lowerb").as_integer_array();
        upperb = cv::Scalar{double(upperb_raw[0]), double(upperb_raw[1]), double(upperb_raw[2])};
        lowerb = cv::Scalar{double(lowerb_raw[0]), double(lowerb_raw[1]), double(lowerb_raw[2])};
    };

    /// Detects the center points of orange traffic cones.
    std::vector<cv::Point2d> detect_objects(const cv::Mat& rgb_mat) override;

    virtual ~ConeDetectionStrategy() = default;
};
