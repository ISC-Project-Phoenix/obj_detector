#include "obj_detector/ObjDetectorNode_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

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

    // TF setup
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listen = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
}

void ObjDetectorNode::process_image(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                                    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    if (!this->rgb_model.initialized()) {
        RCLCPP_INFO(this->get_logger(), "Camera info not received, dropping");
        return;
    }

    // Read as bgr8 to avoid cones being blue in sim
    auto cv_rgb = cv_bridge::toCvShare(rgb, "bgr8");
    auto rgb_mat_distort = cv_rgb->image;

    // Rectify the image, else wide angle cameras will go OOB and segfault
    cv::Mat rgb_mat{};
    this->rgb_model.rectifyImage(rgb_mat_distort, rgb_mat);

    auto cv_depth = cv_bridge::toCvShare(depth);
    auto depth_mat = cv_depth->image;

    // First detect location of objects in pixel space
    auto object_locations = this->detect_objects(rgb_mat);
    // Then project to camera space
    auto poses = this->project_to_world(object_locations, depth_mat);

    this->point_pub->publish(poses);
}

std::vector<cv::Point2d> ObjDetectorNode::detect_objects(const cv::Mat& rgb_mat) {
    // Copy to gpu
    auto rgb = rgb_mat.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_SHARED_MEMORY);

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
    cv::Scalar upperb = cv::Scalar(25, 255, 255);
    cv::Scalar lowerb = cv::Scalar(1, 120, 130);

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

        // Threshold for minimum contour area. Only a masked object of 1500 or more pixels will return a center pixel
        double areaThreshold = 1500.0 / (1280 * 720);

        // Checks if Masked Object is greater than areaThreshold.
        if (M.m00 > areaThreshold * (rgb.rows * rgb.cols))  //M.m00 = total number of white pixels in contour.
        {
            //Uses moment values to find the center pixel (x,y)
            cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);

            // Calculate the bounding rectangle around the contour
            cv::Rect boundingRect = cv::boundingRect(contour);

            // Calculate the aspect ratio of the bounding rectangle
            double aspect_ratio = (double)boundingRect.width / boundingRect.height;

            // Calculate the ratio of the contour area to its arc length
            double area_perimeter_ratio = cv::contourArea(contour) / cv::arcLength(contour, true);

            // Check the properties, compare with the typical properties of a cone
            //CV_TEAM - Calculate specific values for the type of cones we will use, these values below are general TODO
            if (aspect_ratio > 0.5 && aspect_ratio < 1.5 && area_perimeter_ratio > 0.20) {
                centers.push_back(center);
            }
        }
    }

    // Show debug windows
    if (this->debug) {
        cv::UMat dbg;
        mat_gamma_corrected.copyTo(dbg);
        for (auto& center : centers) {
            cv::circle(dbg, center, 5, cv::Scalar(255, 0, 0), -1);  //Draws blue circle at centers
        }

        cv::imshow("mask", mask);    // Shows image of mask
        cv::imshow("centers", dbg);  // Shows image of mask
        cv::pollKey();               // Lets you see the images above.
    }

    return centers;
}

geometry_msgs::msg::PoseArray ObjDetectorNode::project_to_world(const std::vector<cv::Point2d>& object_locations,
                                                                const cv::Mat& depth) {
    geometry_msgs::msg::PoseArray poses{};
    poses.header.frame_id = "mid_cam_link";  //TODO param

    // Rotation that rotates left 90 and backwards 90.
    // This converts from camera coordinates in OpenCV to ROS coordinates
    tf2::Quaternion optical_to_ros{};
    optical_to_ros.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    for (const cv::Point2d& center : object_locations) {
        if (!this->rgb_model.initialized()) {
            continue;
        }

        // Project pixel to camera space
        auto ray = this->rgb_model.projectPixelTo3dRay(center);
        // The oak-d uses shorts in mm, sim uses f32 in m
        float dist = depth.type() == 2 ? (float)depth.at<short>(center) / 1000 : depth.at<float>(center);

        // If depth unavailable, then skip
        if (dist == INFINITY || dist >= 10 || dist <= 0) {
            continue;
        }

        // Just resize the ray to be a vector at the distance of the depth pixel. This is in camera space
        auto point_3d = dist / cv::norm(ray) * ray;

        // Convert from camera space to ros coordinates ("World" but wrt camera mount)
        tf2::Vector3 tf_vec{point_3d.x, point_3d.y, point_3d.z};
        auto world_vec = tf2::quatRotate(optical_to_ros, tf_vec);

        geometry_msgs::msg::Pose p{};
        p.position.x = world_vec.x();
        p.position.y = world_vec.y();
        p.position.z = world_vec.z();
        poses.poses.push_back(p);
    }

    poses.header.stamp = this->get_clock()->now();
    return poses;
}