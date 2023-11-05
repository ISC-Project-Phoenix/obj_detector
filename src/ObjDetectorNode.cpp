#include "obj_detector/ObjDetectorNode_node.hpp"
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

int main(int argc, char **argv) {

    //This code below was taken from python, but I want to code it in c++





    // Setup runtime
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add nodes to executor
    auto node = std::make_shared<ObjDetectorNode>(options);
    exec.add_node(node);

    // Run
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
