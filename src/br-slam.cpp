#include "br-slam.h"

// Constructor for the Bytes SLAM node
BytesSlam::BytesSlam() : Node("bytes_slam"),
left_image_sub_(this, "stereo/left/resize/image"),
left_image_info_sub_(this, "stereo/left/resize/camera_info"),
right_image_sub_(this, "stereo/right/resize/image"),
right_image_info_sub_(this, "stereo/right/resize/camera_info"),
slam(dynamic_cast<Node*>(this), "/home/michael")
{
    RCLCPP_INFO(this->get_logger(), "Beginning Bytes SLAM");

    sync_.reset(new Sync_(sync_policy_(10), left_image_sub_, left_image_info_sub_, right_image_sub_, right_image_info_sub_));
    sync_->registerCallback(&BytesSlam::stereo_cb, this);

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&BytesSlam::imu_cb, this, std::placeholders::_1));
    wheel_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("mobile_base_controller/odom", 10, std::bind(&BytesSlam::wheel_odom_cb, this, std::placeholders::_1));
    altimeter_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("altimeter/pose", 10, std::bind(&BytesSlam::altimeter_cb, this, std::placeholders::_1));
}

// Callback for the stereo camera pair called on synchronization with left and right images
void BytesSlam::stereo_cb(const std::shared_ptr<sensor_msgs::msg::Image>& left, const std::shared_ptr<sensor_msgs::msg::CameraInfo>& left_info,
                          const std::shared_ptr<sensor_msgs::msg::Image>& right, const std::shared_ptr<sensor_msgs::msg::CameraInfo>& right_info) {
    try {
        left_cv_ptr = cv_bridge::toCvShare(left, "mono8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try {
        right_cv_ptr = cv_bridge::toCvShare(right, "mono8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    image_geometry::StereoCameraModel stereo_camera_model;
    stereo_camera_model.fromCameraInfo(left_info, right_info);
    slam.Track(left_cv_ptr->image, right_cv_ptr->image, stereo_camera_model, (static_cast<double>(left_info->header.stamp.sec) + static_cast<double>(left_info->header.stamp.nanosec)/1e9));
}

void BytesSlam::imu_cb(const sensor_msgs::msg::Imu::SharedPtr data) {
    imu_data = data;
}

void BytesSlam::wheel_odom_cb(const nav_msgs::msg::Odometry::SharedPtr data) {
    wheel_odom_data = data;
}

void BytesSlam::altimeter_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data) {
    altimeter_data = data;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BytesSlam>());
    rclcpp::shutdown();
}
