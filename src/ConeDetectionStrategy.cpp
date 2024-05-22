#include "obj_detector/ConeDetectionStrategy.hpp"

std::vector<cv::Point2d> ConeDetectionStrategy::detect_objects(const cv::Mat& rgb_mat) {
    // Copy to gpu
    auto rgb = rgb_mat.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);


// Split the channels
    std::vector<cv::UMat> channels;
    cv::split(rgb, channels); // channels[0] is blue, channels[1] is green, channels[2] is red

// Apply custom processing:
// For instance, highlight red/orange by dividing red by green, considering orange has higher red and lower green
    cv::UMat red_highlighted;
    cv::divide(channels[2], channels[1], red_highlighted); // You might need to normalize and scale this for better results

// Threshold the image to create a binary mask where the red/orange areas are white
    cv::UMat mask_orange;
    double thresh_val = 1.5; // Determine the best threshold value through experimentation
    cv::threshold(red_highlighted, mask_orange, thresh_val, 255, cv::THRESH_BINARY);

// Enhance contrast if necessary (using histogram equalization, for example)
// cv::equalizeHist(mask_orange, mask_orange); // Uncomment and use if needed

// Merge the modified channel back or leave the grayscale as is for further processing
    cv::UMat image_with_highlighted_cones;
    //if (false) { // If you want to merge the highlighted red back into the BGR image, set this to true
    channels[2] = red_highlighted; // Replace the red channel with our processed channel
    cv::merge(channels, image_with_highlighted_cones);


// Now convert this image to HSV and apply cv::inRange() to get the final mask
    cv::UMat hsv;
    cv::cvtColor(image_with_highlighted_cones, hsv, cv::COLOR_BGR2HSV);

    // Define the structuring element for the morphological operations (Required for dilation operation)
    int morphSize = 1;  // change this as per requirement
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morphSize + 1, 2 * morphSize + 1),
                                                cv::Point(morphSize, morphSize));

    // Perform dilation
    cv::UMat mask_dilated;
    cv::dilate(mask_orange, mask_dilated, element);

    // Find contours (Boundary lines of each masked area)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_orange, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Code to calculate and annotate centroids (Center of each masked area)
    std::vector<cv::Point2d> centers;
    for (auto& contour : contours) {
        // Calculate moments for each contour
        cv::Moments M = cv::moments(contour);

        // Checks if Masked Object has more pixels than our minimum (filters small objects)
        if (M.m00 > (this->area_threshold / (720 * 1280)) *
                        (rgb.rows * rgb.cols))  //M.m00 = total number of white pixels in contour.
        {
            //Uses moment values to find the center pixel (x,y)
            cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);

            // Calculate the bounding rectangle around the contour
            cv::Rect boundingRect = cv::boundingRect(contour);

            // Calculate the aspect ratio of the bounding rectangle
            double aspect_ratio = (double)boundingRect.width / boundingRect.height;

            // Calculate the ratio of the contour area to its arcLength(perimeter)
            double area_perimeter_ratio_actual = cv::contourArea(contour) / cv::arcLength(contour, true);

            // Check the properties, compare with the typical properties of a cone
            auto ar_min = this->aspect_ratio_threshold_min;
            auto ar_max = this->aspect_ratio_threshold_max;
            auto ap_ratio = this->area_perimeter_ratio;

            if (aspect_ratio > ar_min && aspect_ratio < ar_max && area_perimeter_ratio_actual > ap_ratio) {
                centers.push_back(center);
            }
        }
    }

    // Show debug windows
    if (this->debug) {
        cv::UMat dbg = rgb.clone();

        for (auto& center : centers) {
            cv::circle(dbg, center, 5, cv::Scalar(255, 0, 0), -1);  //Draws blue circle at centers
        }

        cv::imshow("mask", mask_orange);  // Shows image of mask
        cv::imshow("centers", dbg);        // Shows image of mask
        cv::pollKey();                     // Lets you see the images above.
    }

    return centers;
}