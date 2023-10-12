#include "obj_detector/ObjDetectorNode_node.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// For _1
using namespace std::placeholders;

ObjDetectorNode::ObjDetectorNode(const rclcpp::NodeOptions& options) : Node("ObjDetectorNode", options) {
    // Either "compressed" or "raw"
    std::string transport_type = this->declare_parameter("transport_type", "raw");

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
    auto cv_rgb = cv_bridge::toCvShare(rgb);
    auto rgb_mat = cv_rgb->image;

    // First detect location of objects in pixel space
    auto object_locations = this->detect_objects(rgb_mat);
    // Then project to camera space
    auto poses = this->project_to_world(object_locations, depth);

    this->point_pub->publish(poses);
}

std::vector<cv::Scalar> ObjDetectorNode::detect_objects(const cv::Mat& rgb) {
    //TODO detect
}

geometry_msgs::msg::PoseArray ObjDetectorNode::project_to_world(const std::vector<cv::Scalar>& object_locations,
                                                                const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    //TODO project
}