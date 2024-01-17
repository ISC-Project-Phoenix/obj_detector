//
// Created by jaredwensley on 1/16/24.
//

#pragma once

class DetectionStrategy {
public:

    // TODO      This is probably wrong below
    virtual std::vector<detect> detect_object_centers(const cv::Mat& rgb_Mat);

};