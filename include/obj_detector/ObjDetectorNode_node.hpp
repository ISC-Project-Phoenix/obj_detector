#pragma once

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;

class ObjDetectorNode : public rclcpp::Node {
private:
    // To sync rgb and depth images from compressed streams
    image_transport::SubscriberFilter rgb_syncer;
    image_transport::SubscriberFilter depth_syncer;
    message_filters::Synchronizer<MySyncPolicy> sync;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub;

public:
    ObjDetectorNode(const rclcpp::NodeOptions& options);

    /// Synced images callback
    void process_image(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth);
};
