#pragma once

#include "opencv2/opencv.hpp"
#include "vector"

class IDetectionStrategy {
public:
    /// Returns the u,v pixel locations of every detected object in the unrectified image.
    virtual std::vector<cv::Point2d> detect_objects(const cv::Mat& rgb_mat) = 0;
};