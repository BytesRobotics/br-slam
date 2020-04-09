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
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cv_bridge/cv_bridge.h>

#include "System.h"


class BytesSlam: public rclcpp::Node{
public:
    BytesSlam();
private:
    // Handle subscribing to stereo image topics
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_image_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_image_info_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr altimeter_sub;

    // The follow is synchronized with images to provide optimal SLAM. Images should be publishing at 15fps
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
                                               sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> sync_policy_;
    typedef message_filters::Synchronizer<sync_policy_> Sync_;
    std::shared_ptr<Sync_> sync_;

    void stereo_cb(const std::shared_ptr<sensor_msgs::msg::Image>& left, const std::shared_ptr<sensor_msgs::msg::CameraInfo>& left_info,
                   const std::shared_ptr<sensor_msgs::msg::Image>& right, const std::shared_ptr<sensor_msgs::msg::CameraInfo>& right_info);

    // CV bridge for converting ROS image topics to Opencv topics
    cv_bridge::CvImageConstPtr right_cv_ptr;
    cv_bridge::CvImageConstPtr left_cv_ptr;

    //Standard callbacks for IMU, wheel odom, altimeter @ 30Hz
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr data);
    sensor_msgs::msg::Imu::SharedPtr imu_data;
    void wheel_odom_cb(const nav_msgs::msg::Odometry::SharedPtr data);
    nav_msgs::msg::Odometry::SharedPtr wheel_odom_data;
    void altimeter_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data);
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr altimeter_data;

    // Bytes ORB SLAM system
    System slam;
};

#endif // BR_SLAM_BR_SLAM_H
