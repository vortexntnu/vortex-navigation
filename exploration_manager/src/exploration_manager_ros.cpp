#include <exploration_manager/exploration_manager_ros.hpp>
#include <spdlog/spdlog.h>

using std::placeholders::_1;

ExplorationManagerNode::ExplorationManagerNode(const rclcpp::NodeOptions& options)
    : Node("exploration_manager_node", options) {

    initialize_mapper_params();

    this->declare_parameter<bool>("enu_to_ned");

    odom_frame_ =
        this->declare_parameter<std::string>("odom_frame");

    map_frame_ =
        this->declare_parameter<std::string>("map_frame");

    optical_frame_ =
        this->declare_parameter<std::string>("optical_frame");

    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    auto map_to_odom_tf = compute_map_odom_transform();

    tf_broadcaster_->sendTransform(map_to_odom_tf);

    Eigen::Affine3f eigen_transform = tf2::transformToEigen(map_to_odom_tf.transform).cast<float>();

    map_to_odom_tf_ = eigen_transform.matrix();

    std::chrono::duration<int> buffer_timeout(1);
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
        
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    auto depth_image_sub_topic = this->declare_parameter<std::string>("depth_image_sub_topic");

    depth_sub_.subscribe(this, depth_image_sub_topic, qos.get_rmw_qos_profile());
        
    depth_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::Image>>(
        depth_sub_, *tf_buffer_, map_frame_, 10, 
        this->get_node_logging_interface(), this->get_node_clock_interface());
        
    depth_filter_->registerCallback(std::bind(&ExplorationManagerNode::depth_image_callback, this, _1));
    
    auto camera_info_sub_topic = this->declare_parameter<std::string>("camera_info_sub_topic");
        
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_sub_topic, qos,
        std::bind(&ExplorationManagerNode::camera_info_callback, this, _1));

    auto point_cloud_pub_topic = this->declare_parameter<std::string>("voxelcloud_pub_topic");
    
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        point_cloud_pub_topic, 10);

    auto marker_pub_topic = this->declare_parameter<std::string>("camera_view_visualization_pub_topic");

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_pub_topic, qos);
    
    auto timer_period_ms = this->declare_parameter<int>("timer_period_ms");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&ExplorationManagerNode::timer_callback, this));

    // pointcloud_slice_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    //     "point_cloud_slice", qos);

    // pointcloud_esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    //     "point_cloud_esdf", qos);
}

void ExplorationManagerNode::initialize_mapper_params() {
    MapperParams mapper_params;
    mapper_params.chunk_capacity = this->declare_parameter<int>("voxel_mapping.chunk_capacity");
    mapper_params.resolution = this->declare_parameter<double>("voxel_mapping.grid_resolution");
    mapper_params.min_depth = this->declare_parameter<double>("voxel_mapping.min_depth");
    mapper_params.max_depth = this->declare_parameter<double>("voxel_mapping.max_depth");
    mapper_params.log_odds_occupied = this->declare_parameter<int>("voxel_mapping.log_odds_occupied_update");
    mapper_params.log_odds_free = this->declare_parameter<int>("voxel_mapping.log_odds_free_update");
    mapper_params.log_odds_min = this->declare_parameter<int>("voxel_mapping.log_odds_min");
    mapper_params.log_odds_max = this->declare_parameter<int>("voxel_mapping.log_odds_max");
    mapper_params.occupancy_threshold = this->declare_parameter<int>("voxel_mapping.occupancy_threshold");
    mapper_params.free_threshold = this->declare_parameter<int>("voxel_mapping.free_threshold");

    mapper_params_ = mapper_params;

    mapper_ = std::make_unique<voxel_mapping::VoxelMapping>(mapper_params_.chunk_capacity, mapper_params_.resolution, mapper_params_.min_depth, mapper_params_.max_depth, mapper_params_.log_odds_occupied, mapper_params_.log_odds_free, mapper_params_.log_odds_min, mapper_params_.log_odds_max, mapper_params_.occupancy_threshold, mapper_params_.free_threshold);
}

geometry_msgs::msg::TransformStamped ExplorationManagerNode::compute_map_odom_transform() {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->get_clock()->now();
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;

    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;

    if (this->get_parameter("enu_to_ned").as_bool()) {
        tf2::Quaternion q1;
        q1.setRPY(M_PI, 0.0, M_PI_2);
        map_to_odom.transform.rotation.w = q1.w();
        map_to_odom.transform.rotation.x = q1.x();
        map_to_odom.transform.rotation.y = q1.y();
        map_to_odom.transform.rotation.z = q1.z();
    } else {
        tf2::Quaternion q2;
        q2.setRPY(0.0, 0.0, 0.0);
        map_to_odom.transform.rotation.w = q2.w();
        map_to_odom.transform.rotation.x = q2.x();
        map_to_odom.transform.rotation.y = q2.y();
        map_to_odom.transform.rotation.z = q2.z();
    }

    return map_to_odom;
}
       
void ExplorationManagerNode::depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!camera_info_received_) {
        return;
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(map_frame_, optical_frame_, msg->header.stamp);
    } catch (tf2::TransformException &ex) {
        spdlog::warn("Could not transform depth image: {}", ex.what());
        return;
    }
    Eigen::Affine3f eigen_transform = tf2::transformToEigen(transform.transform).cast<float>();
    Eigen::Matrix4f T = eigen_transform.matrix();

    cam_transform_ = T;

    try {
        const float* tf_ptr = reinterpret_cast<const float*>(T.data());
        const float* depth_ptr = reinterpret_cast<const float*>(msg->data.data());
        mapper_->integrate_depth(depth_ptr, tf_ptr);
    } catch (const std::exception &e) {
        spdlog::error("Error integrating depth image: {}", e.what());
    }

    last_aabb_ = mapper_->get_current_aabb();
    publish_aabb_marker(last_aabb_);
    voxel_mapping::Frustum frustum = mapper_->get_frustum();
    publish_frustum_marker(frustum);

}

void ExplorationManagerNode::timer_callback() {
    if (!camera_info_received_) {
        return;
    }
    voxel_mapping::AABB aabb = last_aabb_;

    if (aabb.size.x == 0 || aabb.size.y == 0 || aabb.size.z == 0) {
        spdlog::warn("AABB size is zero, skipping processing.");
        return;
    }
    std::vector<int> chunk;
    try {
        chunk = mapper_->get_3d_block(aabb);
    } catch (const std::exception &e) {
        spdlog::error("Error getting 3D block: {}", e.what());
        return;
    }

    if (chunk.empty()) {
        spdlog::warn("No data available for the AABB, skipping publishing.");
        return;
    }
    publish_3d_chunk(chunk, aabb);
}

void ExplorationManagerNode::publish_3d_chunk(const std::vector<int>& chunk, voxel_mapping::AABB aabb) {
    struct Point {
        float x, y, z, intensity;
    };
    std::vector<Point> points_to_publish;

    int aabb_min_x = aabb.min_corner_index.x;
    int aabb_min_y = aabb.min_corner_index.y;
    int aabb_min_z = aabb.min_corner_index.z;
    int aabb_size_x = aabb.size.x;
    int aabb_size_y = aabb.size.y;
    int aabb_size_z = aabb.size.z;

    double grid_resolution = mapper_params_.resolution;
    int occupancy_threshold = mapper_params_.occupancy_threshold;

    for (int z = 0; z < aabb_size_z; ++z) {
        for (int y = 0; y < aabb_size_y; ++y) {
            for (int x = 0; x < aabb_size_x; ++x) {
                size_t idx = z * (aabb_size_x * aabb_size_y) + y * aabb_size_x + x;
                if (idx < chunk.size()) {
                    int value = chunk[idx];
                    if (value > occupancy_threshold) {
                        Point p;
                        p.x = static_cast<float>(x + aabb_min_x) * grid_resolution;
                        p.y = static_cast<float>(y + aabb_min_y) * grid_resolution;
                        p.z = static_cast<float>(z + aabb_min_z) * grid_resolution;
                        p.intensity = static_cast<float>(value);
                        points_to_publish.push_back(p);
                    }
                }
            }
        }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    modifier.resize(points_to_publish.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

    for (const auto& point : points_to_publish) {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        *iter_intensity = point.intensity;

        ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
    }

    point_cloud_pub_->publish(cloud_msg);
}

void ExplorationManagerNode::publish_aabb_marker(const voxel_mapping::AABB& aabb) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "aabb";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    float resolution = mapper_params_.resolution;

    Eigen::Vector3f min_corner_world(
        aabb.min_corner_index.x * resolution,
        aabb.min_corner_index.y * resolution,
        aabb.min_corner_index.z * resolution
    );

    Eigen::Vector3f dimensions_world(
        aabb.size.x * resolution,
        aabb.size.y * resolution,
        aabb.size.z * resolution
    );

    Eigen::Vector3f center = min_corner_world + (dimensions_world / 2.0f);

    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();

    marker.pose.orientation.w = 1.0;

    marker.scale.x = dimensions_world.x();
    marker.scale.y = dimensions_world.y();
    marker.scale.z = dimensions_world.z();

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.3;

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    marker_pub_->publish(marker);
}

void ExplorationManagerNode::publish_frustum_marker(const voxel_mapping::Frustum& frustum) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "frustum";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.02;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    std::vector<voxel_mapping::Vec3f> corners;
    corners.reserve(8);

    // Near plane corners (indices 0-3)
    corners.push_back(frustum.near_plane.bl);
    corners.push_back(frustum.near_plane.br);
    corners.push_back(frustum.near_plane.tr);
    corners.push_back(frustum.near_plane.tl);

    // Far plane corners (indices 4-7)
    corners.push_back(frustum.far_plane.bl);
    corners.push_back(frustum.far_plane.br);
    corners.push_back(frustum.far_plane.tr);
    corners.push_back(frustum.far_plane.tl);

    auto add_line = [&marker](const voxel_mapping::Vec3f& p1, const voxel_mapping::Vec3f& p2) {
        geometry_msgs::msg::Point point1, point2;
        point1.x = p1.x; point1.y = p1.y; point1.z = p1.z;
        point2.x = p2.x; point2.y = p2.y; point2.z = p2.z;
        marker.points.push_back(point1);
        marker.points.push_back(point2);
    };

    // Draw near plane
    add_line(corners[0], corners[1]);
    add_line(corners[1], corners[2]);
    add_line(corners[2], corners[3]);
    add_line(corners[3], corners[0]);

    // Draw far plane
    add_line(corners[4], corners[5]);
    add_line(corners[5], corners[6]);
    add_line(corners[6], corners[7]);
    add_line(corners[7], corners[4]);

    // Draw connecting lines
    add_line(corners[0], corners[4]);
    add_line(corners[1], corners[5]);
    add_line(corners[2], corners[6]);
    add_line(corners[3], corners[7]);

    marker_pub_->publish(marker);
}

void ExplorationManagerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    uint32_t width = msg->width;
    uint32_t height = msg->height;
    float fx = msg->k[0];
    float fy = msg->k[4];
    float cx = msg->k[2];
    float cy = msg->k[5];

    mapper_->set_camera_properties(fx, fy, cx, cy, width, height);

    camera_info_received_ = true;
    spdlog::info("Camera info received: width: {}, height: {}, fx: {}, fy: {}, cx: {}, cy: {}",
                 width, height, fx, fy, cx, cy);
    camera_info_sub_.reset();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ExplorationManagerNode)