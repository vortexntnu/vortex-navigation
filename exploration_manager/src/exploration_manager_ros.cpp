#include <exploration_manager/exploration_manager_ros.hpp>
#include <spdlog/spdlog.h>
#ifdef USE_NVTX
    #include <nvtx3/nvToolsExt.h>
#endif

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

    callback_group_sub_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_timer_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    auto depth_image_sub_topic = this->declare_parameter<std::string>("depth_image_sub_topic");

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_sub_;

    depth_sub_.subscribe(this, depth_image_sub_topic, qos.get_rmw_qos_profile(), sub_options);

    depth_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::Image>>(
        depth_sub_, *tf_buffer_, map_frame_, 10, 
        this->get_node_logging_interface(), this->get_node_clock_interface());
        
    depth_filter_->registerCallback(std::bind(&ExplorationManagerNode::depth_image_callback, this, _1));
    
    auto camera_info_sub_topic = this->declare_parameter<std::string>("camera_info_sub_topic");
        
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_sub_topic, qos,
        std::bind(&ExplorationManagerNode::camera_info_callback, this, _1));

    std::string odom_sub_topic = this->declare_parameter<std::string>("odom_sub_topic");
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_sub_topic, 
        qos, 
        std::bind(&ExplorationManagerNode::odometry_callback, this, _1),
        sub_options);

    auto marker_pub_topic = this->declare_parameter<std::string>("camera_view_visualization_pub_topic");
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_pub_topic, qos);
    
    auto timer_period_ms = this->declare_parameter<int>("timer_period_ms");
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&ExplorationManagerNode::timer_callback, this),
        callback_group_timer_);

    std::string grid_block_pub_topic = this->declare_parameter<std::string>("grid_block_pub_topic");
    grid_block_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(grid_block_pub_topic, qos);
    std::string edt_block_pub_topic = this->declare_parameter<std::string>("edt_block_pub_topic");
    edt_block_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(edt_block_pub_topic, qos);
    std::string grid_slices_pub_topic = this->declare_parameter<std::string>("grid_slices_pub_topic");
    grid_slices_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(grid_slices_pub_topic, qos);
    std::string edt_slices_pub_topic = this->declare_parameter<std::string>("edt_slices_pub_topic");
    edt_slices_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(edt_slices_pub_topic, qos);
}

void ExplorationManagerNode::initialize_mapper_params() {
    voxel_mapping::VoxelMappingParams mapper_params;
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
    mapper_params.edt_max_distance = this->declare_parameter<int>("voxel_mapping.edt_max_distance");

    mapper_params_ = mapper_params;

    mapper_ = std::make_unique<voxel_mapping::VoxelMapping>(mapper_params_);
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

void ExplorationManagerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    orca_z_pos_ = -msg->pose.pose.position.z;
}
       
void ExplorationManagerNode::depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!camera_info_received_) {
        return;
    }
    #ifdef USE_NVTX
        nvtxRangePushA("Depth Image Callback");
    #endif
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

    #ifdef USE_NVTX
        nvtxRangePop(); // Depth Image Callback
    #endif
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
    if (map_query_count_ % 10 == 0) {
        mapper_->query_free_chunk_capacity();
    }
    map_query_count_++;

    if (grid_block_pub_->get_subscription_count() > 0) {
        try {
            #ifdef USE_NVTX
                nvtxRangePushA("Block Extraction (Total)");
                nvtxRangePushA("Block Extraction (Launch)");
            #endif

            voxel_mapping::ExtractionResult result = mapper_->extract_grid_block(aabb);

            #ifdef USE_NVTX
                nvtxRangePop(); // Block Extraction (Launch)
            #endif

            result.wait();

            #ifdef USE_NVTX
                nvtxRangePop(); // Block Extraction (Total)
            #endif

            const int* block_data = result.data<int>();
                
            if (block_data != nullptr) {
                publish_grid_block(block_data, result.size_bytes(), aabb);
            } else {
                spdlog::warn("No data available for the AABB, skipping publishing. "
                             "Check for type mismatch or empty result.");
            }
        } catch (const std::exception &e) {
            spdlog::error("Error getting 3D block: {}", e.what());
        }
    }

    if (edt_block_pub_->get_subscription_count() > 0) {
        try {
            #ifdef USE_NVTX
                nvtxRangePushA("EDT Block Extraction (Total)");
                nvtxRangePushA("EDT Block Extraction (Launch)");
            #endif

            voxel_mapping::ExtractionResult result = mapper_->extract_edt_block(aabb);

            #ifdef USE_NVTX
                nvtxRangePop(); // EDT Block Extraction (Launch)
            #endif

            result.wait();

            #ifdef USE_NVTX
                nvtxRangePop(); // EDT Block Extraction (Total)
            #endif
            const int* edt_block_data = result.data<int>();
            
            if (edt_block_data != nullptr) {
                publish_edt_block(edt_block_data, result.size_bytes(), aabb);
            } else {
                spdlog::warn("EDT block is empty or type is mismatched, skipping publishing.");
            }
        } catch (const std::exception &e) {
            spdlog::error("Error extracting EDT block: {}", e.what());
        }
    }

    if (grid_slices_pub_->get_subscription_count() > 0 || edt_slices_pub_->get_subscription_count() > 0) {
        voxel_mapping::SliceZIndices slice_indices;
        slice_indices.indices[0] = aabb.min_corner_index.z + aabb.size.z / 4;
        slice_indices.indices[1] = aabb.min_corner_index.z + aabb.size.z / 2;
        slice_indices.indices[2] = aabb.min_corner_index.z + aabb.size.z * 3 / 4;
        slice_indices.count = 3;

        if (grid_slices_pub_->get_subscription_count() > 0) {
            try {
                #ifdef USE_NVTX
                    nvtxRangePushA("Slice Extraction (Total)");
                    nvtxRangePushA("Slice Extraction (Launch)");
                #endif

                voxel_mapping::ExtractionResult result = mapper_->extract_grid_slices(aabb, slice_indices);

                #ifdef USE_NVTX
                    nvtxRangePop(); // Slice Extraction (Launch)
                #endif

                result.wait();

                #ifdef USE_NVTX
                    nvtxRangePop(); // Slice Extraction (Total)
                #endif
                const int* grid_slices_data = result.data<int>();
                
                if (grid_slices_data != nullptr) {
                    publish_grid_slices(grid_slices_data, result.size_bytes(), aabb, slice_indices);
                } else {
                    spdlog::warn("Grid slices are empty or type is mismatched, skipping publishing.");
                }
            } catch (const std::exception &e) {
                spdlog::error("Error extracting grid slices: {}", e.what());
            }
        }

        if (edt_slices_pub_->get_subscription_count() > 0) {
            try {
                #ifdef USE_NVTX
                    nvtxRangePushA("EDT Slice Extraction (Total)");
                    nvtxRangePushA("EDT Slice Extraction (Launch)");
                #endif

                voxel_mapping::ExtractionResult result = mapper_->extract_edt_slices(aabb, slice_indices);

                #ifdef USE_NVTX
                    nvtxRangePop(); // EDT Slice Extraction (Launch)
                #endif

                result.wait();

                #ifdef USE_NVTX
                    nvtxRangePop(); // EDT Slice Extraction (Total)
                #endif
                const int* edt_slices_data = result.data<int>();
                
                if (edt_slices_data != nullptr) {
                    publish_edt_slices(edt_slices_data, result.size_bytes(), aabb, slice_indices);
                } else {
                    spdlog::warn("EDT slices are empty or type is mismatched, skipping publishing.");
                }
            } catch (const std::exception &e) {
                spdlog::error("Error extracting EDT slices: {}", e.what());
            }
        }
    }
}

void ExplorationManagerNode::publish_grid_block(const int* block_data, size_t size_bytes, const voxel_mapping::AABB& aabb) {
    if (block_data == nullptr || size_bytes == 0) {
        spdlog::warn("Grid block data is empty, skipping publishing.");
        return;
    }

    sensor_msgs::msg::PointCloud2 grid_msg;
    grid_msg.header.stamp = this->get_clock()->now();
    grid_msg.header.frame_id = map_frame_;
    grid_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(grid_msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    size_t total_points = size_bytes / sizeof(int);
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(grid_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(grid_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(grid_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(grid_msg, "intensity");

    float resolution = mapper_params_.resolution;
    int size_x = aabb.size.x;
    int size_y = aabb.size.y;
    int size_z = aabb.size.z;

    for (int z = 0; z < size_z; ++z) {
        for (int y = 0; y < size_y; ++y) {
            for (int x = 0; x < size_x; ++x) {
                int idx = z * (size_x * size_y) + y * size_x + x;
                if (idx < total_points) {
                    int value = block_data[idx];
                    if (value > mapper_params_.occupancy_threshold) {
                        *iter_x = static_cast<float>(x + aabb.min_corner_index.x) * resolution;
                        *iter_y = static_cast<float>(y + aabb.min_corner_index.y) * resolution;
                        *iter_z = static_cast<float>(z + aabb.min_corner_index.z) * resolution;
                        *iter_intensity = static_cast<float>(value);
                    }
                    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
                }
            }
        }
    }

    grid_block_pub_->publish(grid_msg);
}

void ExplorationManagerNode::publish_edt_block(const int* edt_block_data, size_t size_bytes, const voxel_mapping::AABB& aabb) {
    if (edt_block_data == nullptr || size_bytes == 0) {
        spdlog::warn("EDT block data is empty, skipping publishing.");
        return;
    }

    sensor_msgs::msg::PointCloud2 edt_msg;
    edt_msg.header.stamp = this->get_clock()->now();
    edt_msg.header.frame_id = map_frame_;
    edt_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(edt_msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    size_t total_points = size_bytes / sizeof(int);
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(edt_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(edt_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(edt_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(edt_msg, "intensity");

    float resolution = mapper_params_.resolution;
    int size_x = aabb.size.x;
    int size_y = aabb.size.y;
    int size_z = aabb.size.z;

    for (int z = 0; z < size_z; ++z) {
        for (int y = 0; y < size_y; ++y) {
            for (int x = 0; x < size_x; ++x) {
                int idx = z * (size_x * size_y) + y * size_x + x;
                if (idx < total_points) {
                    int value = edt_block_data[idx];
                    if (value > 0) {
                        *iter_x = static_cast<float>(x + aabb.min_corner_index.x) * resolution;
                        *iter_y = static_cast<float>(y + aabb.min_corner_index.y) * resolution;
                        *iter_z = static_cast<float>(z + aabb.min_corner_index.z) * resolution;
                        *iter_intensity = static_cast<float>(value);
                    }
                    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
                }
            }
        }
    }

    edt_block_pub_->publish(edt_msg);
}

void ExplorationManagerNode::publish_grid_slices(const int* grid_slices_data, size_t size_bytes, 
                             const voxel_mapping::AABB& aabb, 
                             const voxel_mapping::SliceZIndices& slice_indices) {
    if (grid_slices_data == nullptr || size_bytes == 0) {
        spdlog::warn("Grid slices data is empty, skipping publishing.");
        return;
    }
    sensor_msgs::msg::PointCloud2 grid_slices_msg;
    grid_slices_msg.header.stamp = this->get_clock()->now();
    grid_slices_msg.header.frame_id = map_frame_;
    grid_slices_msg.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(grid_slices_msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    size_t total_points = size_bytes / sizeof(int);
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(grid_slices_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(grid_slices_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(grid_slices_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(grid_slices_msg, "intensity");

    float resolution = mapper_params_.resolution;
    int size_x = aabb.size.x;
    int size_y = aabb.size.y;
    int size_z = slice_indices.count;

    for (int z = 0; z < size_z; ++z) {
        for (int y = 0; y < size_y; ++y) {
            for (int x = 0; x < size_x; ++x) {
                int idx = z * (size_x * size_y) + y * size_x + x;
                if (idx < total_points) {
                    int value = grid_slices_data[idx];
                    if (value > mapper_params_.occupancy_threshold) {
                        *iter_x = static_cast<float>(x + aabb.min_corner_index.x) * resolution;
                        *iter_y = static_cast<float>(y + aabb.min_corner_index.y) * resolution;
                        *iter_z = static_cast<float>(slice_indices.indices[z]) * resolution;
                        *iter_intensity = static_cast<float>(value);
                    }
                    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
                }
            }
        }
    }

    grid_slices_pub_->publish(grid_slices_msg);
}

void ExplorationManagerNode::publish_edt_slices(const int* edt_slices_data, size_t size_bytes,
                            const voxel_mapping::AABB& aabb,
                            const voxel_mapping::SliceZIndices& slice_indices) {
    if (edt_slices_data == nullptr || size_bytes == 0) {
        spdlog::warn("EDT slices data is empty, skipping publishing.");
        return;
    }
    sensor_msgs::msg::PointCloud2 edt_slices_msg;
    edt_slices_msg.header.stamp = this->get_clock()->now();
    edt_slices_msg.header.frame_id = map_frame_;
    edt_slices_msg.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(edt_slices_msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    size_t total_points = size_bytes / sizeof(int);
    modifier.resize(total_points);
    sensor_msgs::PointCloud2Iterator<float> iter_x(edt_slices_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(edt_slices_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(edt_slices_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(edt_slices_msg, "intensity");

    float resolution = mapper_params_.resolution;
    int size_x = aabb.size.x;
    int size_y = aabb.size.y;
    int size_z = slice_indices.count;

    int max_dim_square = std::max(size_x, size_y) * std::max(size_x, size_y);

    for (int z = 0; z < size_z; ++z) {
        for (int y = 0; y < size_y; ++y) {
            for (int x = 0; x < size_x; ++x) {
                int idx = z * (size_x * size_y) + y * size_x + x;
                if (idx < total_points) {
                    int value = edt_slices_data[idx];
                    if (value > 0) {
                        *iter_x = static_cast<float>(x + aabb.min_corner_index.x) * resolution;
                        *iter_y = static_cast<float>(y + aabb.min_corner_index.y) * resolution;
                        *iter_z = static_cast<float>(slice_indices.indices[z]) * resolution;
                        *iter_intensity = static_cast<float>(value);
                    }
                    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
                }
            }
        }
    }

    edt_slices_pub_->publish(edt_slices_msg);
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