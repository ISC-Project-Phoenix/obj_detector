#include "obj_detector/ConeDetectionStrategy.hpp"

std::vector<cv::Point2d> ConeDetectionStrategy::detect_objects(const cv::Mat& rgb_mat) {
    // Copy to gpu
    auto rgb = rgb_mat.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    // Define gamma and create a lookup table
    //gamma will brighten shadows on image
    double gamma = 0.7;  // Value < 1 'Image brightens shadows. Value > 1 Image darkens shadows
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i) p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);

    // Apply gamma correction
    cv::UMat mat_gamma_corrected;
    cv::LUT(rgb, lookUpTable, mat_gamma_corrected);

    // Changes image to HSV (Hue, Sat, value)
    cv::UMat hsv;
    cv::cvtColor(mat_gamma_corrected, hsv, cv::COLOR_BGR2HSV);

    //Setting the HSV values to the color we are masking.
    cv::Scalar upperb = cv::Scalar(15, 255, 255);
    cv::Scalar lowerb = cv::Scalar(0, 130, 130);

    //Applying the above bounds to the gamma_image to create mask
    cv::UMat mask;
    cv::inRange(hsv, lowerb, upperb, mask);

    // Define the structuring element for the morphological operations (Required for dilation operation)
    int morphSize = 3;  // change this as per requirement
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morphSize + 1, 2 * morphSize + 1),
                                                cv::Point(morphSize, morphSize));

    // Perform dilation
    cv::UMat mask_dilated;
    cv::dilate(mask, mask_dilated, element);

    // Find contours (Boundary lines of each masked area)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_dilated, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Code to calculate and annotate centroids (Center of each masked area)
    std::vector<cv::Point2d> centers;
    for (auto& contour : contours) {
        // Calculate moments for each contour
        cv::Moments M = cv::moments(contour);

        // Checks if Masked Object has more pixels than our minimum (filters small objects)
        if (M.m00 > (this->parent.get_parameter("area_threshold").as_double() / (720 * 1280)) *
                        (rgb.rows * rgb.cols))  //M.m00 = total number of white pixels in contour.
        {
            //Uses moment values to find the center pixel (x,y)
            cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);

            // Calculate the bounding rectangle around the contour
            cv::Rect boundingRect = cv::boundingRect(contour);

            // Calculate the aspect ratio of the bounding rectangle
            double aspect_ratio = (double)boundingRect.width / boundingRect.height;

            // Calculate the ratio of the contour area to its arcLength(perimeter)
            double area_perimeter_ratio = cv::contourArea(contour) / cv::arcLength(contour, true);

            // Check the properties, compare with the typical properties of a cone
            auto ar_min = this->parent.get_parameter("aspect_ratio_threshold_min").as_double();
            auto ar_max = this->parent.get_parameter("aspect_ratio_threshold_max").as_double();
            auto ap_ratio = this->parent.get_parameter("area_perimeter_ratio").as_double();

            if (aspect_ratio > ar_min && aspect_ratio < ar_max && area_perimeter_ratio > ap_ratio) {
                centers.push_back(center);
            }
        }
    }

    // Show debug windows
    if (this->parent.get_parameter("debug").as_bool()) {
        cv::UMat dbg;
        mat_gamma_corrected.copyTo(dbg);
        for (auto& center : centers) {
            cv::circle(dbg, center, 5, cv::Scalar(255, 0, 0), -1);  //Draws blue circle at centers
        }

        cv::imshow("mask", mask_dilated);  // Shows image of mask
        cv::imshow("centers", dbg);        // Shows image of mask
        cv::pollKey();                     // Lets you see the images above.
    }

    return centers;
}