#ifndef EXPLORATION_MANAGER_ROS_HPP
#define EXPLORATION_MANAGER_ROS_HPP

#include "voxel-mapping/voxel_mapping.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ExplorationManagerNode : public rclcpp::Node {
   public:
    ExplorationManagerNode(const rclcpp::NodeOptions& options);

    ~ExplorationManagerNode() {};

    void initialize_mapper_params();

    void depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void publish_aabb_marker(const voxel_mapping::AABB& aabb);

    void publish_frustum_marker(const voxel_mapping::Frustum& frustum);

    void publish_3d_chunk(const std::vector<int>& chunk, const voxel_mapping::AABB& aabb);

    void publish_esdf_slice(const std::vector<int>& esdf_slice, const voxel_mapping::AABB& aabb);

    geometry_msgs::msg::TransformStamped compute_map_odom_transform();

    void timer_callback();

   private:
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
   message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
   std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::Image>> depth_filter_;
   
   rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
   
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
      
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_esdf_pub_;
   
   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
   
   rclcpp::TimerBase::SharedPtr timer_;
   
   bool camera_info_received_ = false;
    std::string odom_frame_;
    std::string map_frame_;
    std::string optical_frame_;
    float orca_z_pos_ = 0.0f;
    size_t map_query_count_ = 0;

    std::unique_ptr<voxel_mapping::VoxelMapping> mapper_;
    voxel_mapping::VoxelMappingParams mapper_params_;
    voxel_mapping::AABB last_aabb_;

    Eigen::Matrix4f map_to_odom_tf_;
    Eigen::Matrix4f cam_transform_;

};

#endif  // EXPLORATION_MANAGER_ROS_HPP