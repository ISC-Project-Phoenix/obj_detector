//
// Created by jaredwensley on 1/16/24.
//

#pragma once

struct ObjDetectorParams{
    // TODO param that changes detect type?
};

class DetectionStrategy {
public:
    // TODO      This is probably wrong below
    virtual DetectOption detect(const cv::Mat& rgb_Mat) = 0;
    virtual ~IDetectionClassifier() = default;
};

class temp : public DetectionStrategy{
public:
    DetectionOption detect(const cv::Mat& rgb_Mat) override;

protected:
private:

    // TODO Something here
};