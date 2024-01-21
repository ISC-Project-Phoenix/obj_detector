#include "detection_cat/DetectionCatNode_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

// For _1
using namespace std::placeholders;

DetectionCatNode::DetectionCatNode(const rclcpp::NodeOptions& opts) : rclcpp::Node("detection_cat", opts) {
    // Random params
    this->declare_parameter<std::string>("common_frame", "mid_cam_link");
    this->declare_parameter<double>("overlap_tolerance", 0.1);

    this->concat_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/object_detection_cat", 5);

    // Sync the two topics
    this->detect1_sub.subscribe(this, "/object_detection1");
    this->detect2_sub.subscribe(this, "/object_detection2");
    this->sync = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), this->detect1_sub,
                                                                               this->detect2_sub);
    this->sync->registerCallback(std::bind(&DetectionCatNode::process_poses, this, _1, _2));

    // TF setup
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listen = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
}

void DetectionCatNode::process_poses(const geometry_msgs::msg::PoseArray::ConstSharedPtr& pose1,
                                     const geometry_msgs::msg::PoseArray::ConstSharedPtr& pose2) {
    geometry_msgs::msg::PoseArray combined{};

    // Transform and concat
    try {
        // Transform all to common frame and concat
        auto pose1_to_com = this->tf_buffer->lookupTransform(this->get_parameter("common_frame").as_string(),
                                                             pose1->header.frame_id, rclcpp::Time{});
        auto pose2_to_com = this->tf_buffer->lookupTransform(this->get_parameter("common_frame").as_string(),
                                                             pose2->header.frame_id, rclcpp::Time{});

        // Add all from first
        for (const auto& item : pose1->poses) {
            geometry_msgs::msg::Pose pose{};
            tf2::doTransform(item, pose, pose1_to_com);

            combined.poses.push_back(pose);
        }
        // Add all from second
        for (const auto& item : pose2->poses) {
            geometry_msgs::msg::Pose pose{};
            tf2::doTransform(item, pose, pose2_to_com);

            combined.poses.push_back(pose);
        }
    } catch (tf2::LookupException& e) {
        RCLCPP_INFO(this->get_logger(), "Failed lookup!");
        return;
    }

    std::vector<size_t> skip_list{};
    geometry_msgs::msg::PoseArray out{};

    // Find all points that are too close to other points
    for (size_t i = 0; i < combined.poses.size(); i++) {
        auto pt1 = combined.poses[i];
        std::vector<double> distance;

        for (size_t j = 0; j < combined.poses.size(); j++) {
            // Skip skipped points
            if (std::find(skip_list.begin(), skip_list.end(), j) != skip_list.end() || j == i) {
                continue;
            }

            auto pt2 = combined.poses[j];
            distance.push_back(std::hypot(pt1.position.x - pt2.position.x, pt1.position.y - pt2.position.y));
        }

        // If any other detection is too close, remove
        for (const auto& item : distance) {
            if (item < 0.1f) {
                skip_list.push_back(i);
                break;
            }
        }
    }

    for (size_t i = 0; i < combined.poses.size(); ++i) {
        if (std::find(skip_list.begin(), skip_list.end(), i) != skip_list.end()) {
            continue;
        }

        out.poses.push_back(combined.poses[i]);
    }

    out.header.frame_id = this->get_parameter("common_frame").as_string();
    out.header.stamp = pose1->header.stamp;

    this->concat_pub->publish(out);
}
