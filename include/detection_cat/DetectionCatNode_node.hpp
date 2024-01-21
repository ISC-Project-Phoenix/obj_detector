#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseArray, geometry_msgs::msg::PoseArray>
    MySyncPolicy;

class DetectionCatNode : public rclcpp::Node {
    // Subs
    message_filters::Subscriber<geometry_msgs::msg::PoseArray> detect1_sub;
    message_filters::Subscriber<geometry_msgs::msg::PoseArray> detect2_sub;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

    // Pubs
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr concat_pub;

    // TF stuff
    std::unique_ptr<tf2_ros::TransformListener> tf_listen;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

public:
    explicit DetectionCatNode(const rclcpp::NodeOptions& opts);

    void process_poses(const geometry_msgs::msg::PoseArray::ConstSharedPtr& pose1,
                       const geometry_msgs::msg::PoseArray::ConstSharedPtr& pose2);
};