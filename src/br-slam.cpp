#include "br-slam.h"

// Constructor for the Bytes SLAM node
BytesSlam::BytesSlam() : Node("bytes_slam"),
left_image_sub_(this, "stereo/left/resize/image"),
right_image_sub_(this, "stereo/right/resize/image"),
sync_(left_image_sub_, right_image_sub_, 10)
{
    sync_.registerCallback(&BytesSlam::stereo_cb, this);
}

// Callback for the stereo camera pair called on synchronization with left and right images
void BytesSlam::stereo_cb(const std::shared_ptr<sensor_msgs::msg::Image>& left, const std::shared_ptr<sensor_msgs::msg::Image>& right) {
//    RCLCPP_INFO(this->get_logger(), left->header.frame_id.c_str());
//    RCLCPP_INFO(this->get_logger(), right->header.frame_id.c_str());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BytesSlam>());
    rclcpp::shutdown();
}
