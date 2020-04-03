//
// Created by michael on 4/2/20.
// This defines the overlaying class that will act as the node that handled the ROS2 interaction with the
// Bytes augmented ORB SLAM2 library
//

#ifndef BR_SLAM_BR_SLAM_H
#define BR_SLAM_BR_SLAM_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


class BytesSlam: public rclcpp::Node{
public:
    BytesSlam();
private:
    // Handle subscribing to stereo image topics
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_;
    void stereo_cb(const std::shared_ptr<sensor_msgs::msg::Image>& left, const std::shared_ptr<sensor_msgs::msg::Image>& right);
};

#endif // BR_SLAM_BR_SLAM_H
