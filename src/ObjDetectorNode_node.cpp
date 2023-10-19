#include "obj_detector/ObjDetectorNode_node.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/ocl.hpp"
#include "opencv2/opencv.hpp"

// For _1
using namespace std::placeholders;

ObjDetectorNode::ObjDetectorNode(const rclcpp::NodeOptions& options) : Node("ObjDetectorNode", options) {
    // Enable opencl acceleration
    cv::ocl::setUseOpenCL(true);
    RCLCPP_INFO(this->get_logger(), "using opencl: %u", cv::ocl::haveOpenCL());

    // Either "compressed" or "raw"
    std::string transport_type = this->declare_parameter("transport_type", "raw");
    // Show debug views or not
    this->debug = this->declare_parameter<bool>("debug", true);

    RCLCPP_INFO(this->get_logger(), "Detecting using transport: %s", transport_type.c_str());

    // Synchronize depth and rgb images, using some transport method
    this->rgb_syncer = std::make_unique<image_transport::SubscriberFilter>(this, "/camera/mid/rgb", transport_type);
    this->depth_syncer = std::make_unique<image_transport::SubscriberFilter>(this, "/camera/mid/depth", "raw");
    this->sync =
        std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), *rgb_syncer, *depth_syncer);

    // Register callback for synced images
    this->sync->registerCallback(std::bind(&ObjDetectorNode::process_image, this, _1, _2));

    // Copy rgb camera info directly into model
    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    // Output detected objects
    this->point_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/object_poses", 5);
}

void ObjDetectorNode::process_image(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                                    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    // Read as bgr8 to avoid cones being blue in sim
    auto cv_rgb = cv_bridge::toCvShare(rgb, "bgr8");
    auto rgb_mat = cv_rgb->image;

    // First detect location of objects in pixel space
    auto object_locations = this->detect_objects(rgb_mat);
    // Then project to camera space
    //auto poses = this->project_to_world(object_locations, depth); TODO add when depth done

    //this->point_pub->publish(poses);
}

std::vector<cv::Point2d> ObjDetectorNode::detect_objects(const cv::Mat& rgb_mat) {
    // Copy to gpu
    auto rgb = rgb_mat.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_SHARED_MEMORY);

    cv::UMat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);  // Changes image to HSV (Hue, Sat, value)

    //Setting the HSV values to the color we are masking.
    cv::Scalar upperb = cv::Scalar(20, 255, 255);
    cv::Scalar lowerb = cv::Scalar(0, 150, 120);

    //Applying the above bounds to the image to create mask
    cv::UMat mask;
    cv::inRange(hsv, lowerb, upperb, mask);

    // Find contours (Boundary lines of each masked area)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Code to calculate and annotate centroids (Center of each masked area)
    std::vector<cv::Point2d> centers;
    for (auto& contour : contours) {
        // Calculate moments for each contour
        cv::Moments M = cv::moments(contour);

        if (M.m00 > 0)  // Avoid division by zero
        {
            cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);

            centers.push_back(center);
        }
    }

    // Show debug windows
    if (this->debug) {
        cv::UMat dbg;
        rgb.copyTo(dbg);
        for (auto& center : centers) {
            cv::circle(dbg, center, 5, cv::Scalar(255, 0, 0), -1);
        }

        cv::imshow("mask", mask);    // Shows image of mask
        cv::imshow("centers", dbg);  // Shows image of mask
        cv::pollKey();               // Lets you see the images above.
    }

    return centers;
}

geometry_msgs::msg::PoseArray ObjDetectorNode::project_to_world(const std::vector<cv::Point2d>& object_locations,
                                                                const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    //TODO project
}