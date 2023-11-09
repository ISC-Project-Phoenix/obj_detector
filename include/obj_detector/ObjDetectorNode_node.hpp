#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include "geometry_msgs/msg/pose_array.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;

class ObjDetectorNode : public rclcpp::Node {
private:
    // To sync rgb and depth images from compressed streams
    std::unique_ptr<image_transport::SubscriberFilter> rgb_syncer;
    std::unique_ptr<image_transport::SubscriberFilter> depth_syncer;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr point_pub;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub;
    /// Geometric model of the rgb camera
    image_geometry::PinholeCameraModel rgb_model;

    bool debug;

    std::unique_ptr<tf2_ros::TransformListener> tf_listen;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

public:
    ObjDetectorNode(const rclcpp::NodeOptions& options);

    /// Synced images callback
    void process_image(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    /// Returns the u,v pixel locations of every detected object in the unrectified image.
    std::vector<cv::Point2d> detect_objects(const cv::Mat& rgb);

    /// Takes a list of u,v pixel locations, and returns a list of those points in the real world (still in camera frame).
    geometry_msgs::msg::PoseArray project_to_world(const std::vector<cv::Point2d>& object_locations,
                                                   const cv::Mat& depth);
    void calc_latency(long ms) const;
};
