#include "obj_detector/ObjDetectorNode_node.hpp"

#include "functional"
#include "opencv2/opencv.hpp"

// For _1
using namespace std::placeholders;

ObjDetectorNode::ObjDetectorNode(const rclcpp::NodeOptions& options)
    : Node("ObjDetectorNode", options),
      // Sync the rgb and depth images, from compressed streams
      rgb_syncer(this, "/camera/mid/rgb", "compressed"),
      depth_syncer(this, "/camera/mid/depth", "compressed"),
      sync(MySyncPolicy(10), rgb_syncer, depth_syncer) {
    // Register callback for synced images
    this->sync.registerCallback(std::bind(&ObjDetectorNode::process_image, this, _1, _2));
    // Output detected objects
    this->point_pub = this->create_publisher<geometry_msgs::msg::Point>("/object_poses", 5);
}

void ObjDetectorNode::process_image(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                                    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    RCLCPP_INFO(this->get_logger(), "Got image!");
    //TODO detect the things
}