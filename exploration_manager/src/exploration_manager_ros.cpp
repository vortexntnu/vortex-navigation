#include <exploration_manager/exploration_manager_ros.hpp>

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

    exploration_manager_.set_map_to_odom_tf(eigen_transform.matrix());
    
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

    auto dvl_altitude_sub_topic = this->declare_parameter<std::string>("dvl_altitude_sub_topic");

    dvl_altitude_sub_ = this->create_subscription<vortex_msgs::msg::DVLAltitude>(
        dvl_altitude_sub_topic, qos,
        std::bind(&ExplorationManagerNode::dvl_altitude_callback, this, _1));

    auto point_cloud_pub_topic = this->declare_parameter<std::string>("point_cloud_pub_topic");
    
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        point_cloud_pub_topic, qos);

    auto marker_pub_topic = this->declare_parameter<std::string>("camera_view_visualization_pub_topic");

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_pub_topic, qos);
    
    auto timer_period_ms = this->declare_parameter<int>("timer_period_ms");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&ExplorationManagerNode::timer_callback, this));
}

void ExplorationManagerNode::initialize_mapper_params() {
    MapperParams mapper_params;
    mapper_params.resolution = this->declare_parameter<double>("voxel_mapping.grid_resolution");
    mapper_params.size_x = this->declare_parameter<int>("voxel_mapping.grid_size_x");
    mapper_params.size_y = this->declare_parameter<int>("voxel_mapping.grid_size_y");
    mapper_params.size_z = this->declare_parameter<int>("voxel_mapping.grid_size_z");
    mapper_params.min_depth = this->declare_parameter<double>("voxel_mapping.min_depth");
    mapper_params.max_depth = this->declare_parameter<double>("voxel_mapping.max_depth");
    mapper_params.log_odds_occupied = this->declare_parameter<double>("voxel_mapping.log_odds_occupied_update");
    mapper_params.log_odds_free = this->declare_parameter<double>("voxel_mapping.log_odds_free_update");
    mapper_params.log_odds_min = this->declare_parameter<double>("voxel_mapping.log_odds_min");
    mapper_params.log_odds_max = this->declare_parameter<double>("voxel_mapping.log_odds_max");
    mapper_params.occupancy_threshold = this->declare_parameter<double>("voxel_mapping.occupancy_threshold");
    mapper_params.free_threshold = this->declare_parameter<double>("voxel_mapping.free_threshold");
    
    exploration_manager_.initialize_mapper(mapper_params);
}

geometry_msgs::msg::TransformStamped ExplorationManagerNode::compute_map_odom_transform() {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->get_clock()->now();
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;

    int size_x = this->get_parameter("voxel_mapping.grid_size_x").as_int();
    int size_y = this->get_parameter("voxel_mapping.grid_size_y").as_int();
    int size_z = this->get_parameter("voxel_mapping.grid_size_z").as_int();
    float resolution = this->get_parameter("voxel_mapping.grid_resolution").as_double();

    float center_x = static_cast<float>((size_x - 1) / 2.0) * resolution;
    float center_y = static_cast<float>((size_y - 1) / 2.0) * resolution;
    float center_z = static_cast<float>((size_z - 1) / 2.0) * resolution;

    map_to_odom.transform.translation.x = center_x;
    map_to_odom.transform.translation.y = center_y;
    map_to_odom.transform.translation.z = center_z;
    
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
        RCLCPP_WARN(this->get_logger(), "Could not transform depth image: %s", ex.what());
        return;
    }
    Eigen::Affine3f eigen_transform = tf2::transformToEigen(transform.transform).cast<float>();
    Eigen::Matrix4f T = eigen_transform.matrix();

    exploration_manager_.set_cam_pos_map_frame(Eigen::Vector3f(transform.transform.translation.x,
                                                              transform.transform.translation.y,
                                                              transform.transform.translation.z));
    exploration_manager_.set_cam_transform(T);

    exploration_manager_.process_depth_image(reinterpret_cast<const float*>(msg->data.data()));

    AABB aabb = exploration_manager_.get_last_aabb();
    publish_aabb_marker(aabb);
    publish_frustum_marker(T);
    
    Eigen::VectorXi aabb_indices = exploration_manager_.get_aabb_indices(aabb);

    std::vector<float> block = exploration_manager_.get_updated_block();

    float grid_resolution = this->get_parameter("voxel_mapping.grid_resolution").as_double();
    int grid_size_x = this->get_parameter("voxel_mapping.grid_size_x").as_int();
    int grid_size_y = this->get_parameter("voxel_mapping.grid_size_y").as_int();
    int grid_size_z = this->get_parameter("voxel_mapping.grid_size_z").as_int();

    int aabb_min_x = aabb_indices[0];
    int aabb_max_x = aabb_indices[1];
    int aabb_min_y = aabb_indices[2];
    int aabb_max_y = aabb_indices[3];
    int aabb_min_z = aabb_indices[4];
    int aabb_max_z = aabb_indices[5];
    size_t aabb_size_x = aabb_max_x - aabb_min_x + 1;
    size_t aabb_size_y = aabb_max_y - aabb_min_y + 1;
    size_t aabb_size_z = aabb_max_z - aabb_min_z + 1;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    modifier.resize(aabb_size_x * aabb_size_y * aabb_size_z);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

    size_t point_count = 0;
    for (size_t x = 0; x < aabb_size_x; ++x) {
        for (size_t y = 0; y < aabb_size_y; ++y) {
            for (size_t z = 0; z < aabb_size_z; ++z) {
                // Z-major index: x * (size_y * size_z) + y * size_z + z
                size_t idx = x * (aabb_size_y * aabb_size_z) + y * aabb_size_z + z;
                if (idx < block.size()) {
                    float value = block[idx];
                    if (value > this->get_parameter("voxel_mapping.occupancy_threshold").as_double()) {
                        *iter_x = static_cast<float>(aabb_min_x + x) * grid_resolution;
                        *iter_y = static_cast<float>(aabb_min_y + y) * grid_resolution;
                        *iter_z = static_cast<float>(aabb_min_z + z) * grid_resolution;
                        *iter_intensity = value;
                        ++iter_x;
                        ++iter_y;
                        ++iter_z;
                        ++iter_intensity;
                        ++point_count;
                    }
                }
            }
        }
    }

    modifier.resize(point_count);
    point_cloud_pub_->publish(cloud_msg);
}

void ExplorationManagerNode::timer_callback() {
    exploration_manager_.exploration_timer_callback();
}

void ExplorationManagerNode::publish_aabb_marker(const AABB& aabb) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "aabb";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3f center = (aabb.min_corner + aabb.max_corner) / 2.0f;
    Eigen::Vector3f dimensions = aabb.max_corner - aabb.min_corner;

    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();

    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker.scale.x = dimensions.x();
    marker.scale.y = dimensions.y();
    marker.scale.z = dimensions.z();

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.3;

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    marker_pub_->publish(marker);
}

void ExplorationManagerNode::publish_frustum_marker(const Eigen::Matrix4f& T) {
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

    std::vector<Eigen::Vector4f> frustum_corners = exploration_manager_.compute_frustum_corners();

    std::vector<Eigen::Vector3f> transformed_corners;
    for (const auto& corner : frustum_corners) {
        Eigen::Vector4f transformed = T * corner;
        transformed_corners.push_back(transformed.head<3>() / transformed.w());
    }

    auto add_line = [&marker](const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
        geometry_msgs::msg::Point point1, point2;
        point1.x = p1.x(); point1.y = p1.y(); point1.z = p1.z();
        point2.x = p2.x(); point2.y = p2.y(); point2.z = p2.z();
        marker.points.push_back(point1);
        marker.points.push_back(point2);
    };

    add_line(transformed_corners[0], transformed_corners[1]);
    add_line(transformed_corners[1], transformed_corners[2]);
    add_line(transformed_corners[2], transformed_corners[3]);
    add_line(transformed_corners[3], transformed_corners[0]);

    add_line(transformed_corners[4], transformed_corners[5]);
    add_line(transformed_corners[5], transformed_corners[6]);
    add_line(transformed_corners[6], transformed_corners[7]);
    add_line(transformed_corners[7], transformed_corners[4]);

    add_line(transformed_corners[0], transformed_corners[4]);
    add_line(transformed_corners[1], transformed_corners[5]);
    add_line(transformed_corners[2], transformed_corners[6]);
    add_line(transformed_corners[3], transformed_corners[7]);

    marker_pub_->publish(marker);
}

void ExplorationManagerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    ImageProperties image_properties;
    image_properties.width = msg->width;
    image_properties.height = msg->height;
    image_properties.min_depth = this->get_parameter("min_depth").as_double();
    image_properties.max_depth = this->get_parameter("max_depth").as_double();
    float fx = msg->k[0];
    float fy = msg->k[4];
    float cx = msg->k[2];
    float cy = msg->k[5];
    image_properties.fx = fx;
    image_properties.fy = fy;
    image_properties.cx = cx;
    image_properties.cy = cy;
    exploration_manager_.set_image_properties(image_properties);
    
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f", fx, fy, cx, cy);
    camera_info_sub_.reset();
}

void ExplorationManagerNode::dvl_altitude_callback(const vortex_msgs::msg::DVLAltitude::SharedPtr msg) {
    exploration_manager_.set_dvl_altitude(msg->altitude);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ExplorationManagerNode)