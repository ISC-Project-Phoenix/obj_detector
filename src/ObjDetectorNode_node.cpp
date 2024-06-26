#include "obj_detector/ObjDetectorNode_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

#include "cv_bridge/cv_bridge.h"
#include "obj_detector/ConeDetectionStrategy.hpp"
#include "opencv2/core/ocl.hpp"

// For _1
using namespace std::placeholders;

ObjDetectorNode::ObjDetectorNode(const rclcpp::NodeOptions& options) : Node("ObjDetectorNode", options) {
    // Declare random parameters we don't store in the class
    this->declare_parameter("camera_frame", "mid_cam_link");
    this->declare_parameter("test_latency", false);

    // Image filtering params
    this->declare_parameter("area_threshold", 0.0);
    this->declare_parameter("aspect_ratio_threshold_min", 0.0);
    this->declare_parameter("aspect_ratio_threshold_max", 100000000.0);
    this->declare_parameter("area_perimeter_ratio", 0.0);
    this->declare_parameter("hsv_upperb", std::vector<int>{15, 255, 255});
    this->declare_parameter("hsv_lowerb", std::vector<int>{0, 130, 130});
    this->declare_parameter("gamma", 0.7);

    // Enable opencl acceleration
    bool use_ocl = this->declare_parameter("use_opencl", true);
    cv::ocl::setUseOpenCL(use_ocl);
    RCLCPP_INFO(this->get_logger(), "using opencl: %u", use_ocl && cv::ocl::haveOpenCL());

    // Either "compressed" or "raw"
    std::string transport_type = this->declare_parameter("transport_type", "raw");
    // Show debug views or not
    this->debug = this->declare_parameter<bool>("debug", false);

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

    // Choose strategies
    this->detection_strat = std::make_unique<ConeDetectionStrategy>(*this);
}

void ObjDetectorNode::process_image(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                                    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    // Start time for latency testing
    auto start_t = std::chrono::steady_clock::now();

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
    auto object_locations = this->detection_strat->detect_objects(rgb_mat);
    // Then project to camera space
    auto poses = this->project_to_world(object_locations, depth_mat);

    this->point_pub->publish(poses);

    // Print e2e latency if desired
    if (this->get_parameter("test_latency").as_bool()) {
        auto delta = std::chrono::steady_clock::now() - start_t;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(delta).count();
        calc_latency(ms);
    }
}

geometry_msgs::msg::PoseArray ObjDetectorNode::project_to_world(const std::vector<cv::Point2d>& object_locations,
                                                                const cv::Mat& depth) {
    geometry_msgs::msg::PoseArray poses{};
    poses.header.frame_id = this->get_parameter(std::string{"camera_frame"}).as_string();

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

void ObjDetectorNode::calc_latency(long ms) const {
    static std::array<uint64_t, 300> measurements;
    static uint64_t index = 0;
    measurements[index] = ms;
    index = index + 1 > measurements.size() - 1 ? 0 : index + 1;

    // Calc statistics
    double mean = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean += (double)measurements[i];
    }
    mean /= (double)index + 1;

    double mean2 = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean2 += std::pow((double)measurements[i], 2);
    }
    mean2 /= (double)index + 1;

    double std_dev = sqrt(mean2 - std::pow(mean, 2));

    RCLCPP_INFO(get_logger(), "Mean: %f Std-dev: %f", mean, std_dev);
}