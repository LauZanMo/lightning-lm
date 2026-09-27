//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include <pcl_conversions/pcl_conversions.h>
#include "core/lightning_math.hpp"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        livox_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/lightning/odom", 10);
    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lightning/cloud", 1);
    map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lightning/map", rclcpp::QoS(1).reliable().transient_local());

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    loc_->SetTFCallback([this](const geometry_msgs::msg::TransformStamped &tf) {
        if (tf_broadcaster_) {
            tf_broadcaster_->sendTransform(tf);
        }

        nav_msgs::msg::Odometry msg;
        msg.header = tf.header;
        msg.child_frame_id = tf.child_frame_id;
        msg.pose.pose.position.x = tf.transform.translation.x;
        msg.pose.pose.position.y = tf.transform.translation.y;
        msg.pose.pose.position.z = tf.transform.translation.z;
        msg.pose.pose.orientation = tf.transform.rotation;
        odom_pub_->publish(msg);
    });

    loc_->SetPointcloudWorldCallback(
        [this](const sensor_msgs::msg::PointCloud2 &msg) { cloud_pub_->publish(msg); });

    const float leaf = yaml.GetValue<float>("system", "map_publish_leaf_size");
    loc_->SetMapCallback([this, leaf](const CloudPtr &cloud) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*math::VoxelGrid(cloud, leaf), msg);
        msg.header.frame_id = "map";
        msg.header.stamp = node_->now();
        map_pub_->publish(msg);
    });

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
