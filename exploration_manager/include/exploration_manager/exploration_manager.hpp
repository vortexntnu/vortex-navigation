#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <voxel_mapping.hpp>
#include <Eigen/Dense>
#include <functional>

struct MapperParams {
    float resolution;
    uint size_x;
    uint size_y;
    uint size_z;
    float min_depth;
    float max_depth;
    float log_odds_occupied;
    float log_odds_free;
    float log_odds_min;
    float log_odds_max;
    float occupancy_threshold;
    float free_threshold;
};

struct AABB {
    Eigen::Vector3f min_corner;
    Eigen::Vector3f max_corner;
};

struct ImageProperties {
    uint width;
    uint height;
    float min_depth;
    float max_depth;
    float fx;
    float fy;
    float cx;
    float cy;
};


class ExplorationManager {
    public:
    /**
     * @brief Construct a new Exploration Manager object.
     */
    ExplorationManager(/* args */);

    /**
     * @brief Destroy the Exploration Manager object.
     */
    ~ExplorationManager();

     /**
     * @brief Initialize the mapper with the given parameters.
     * 
     * @param params Parameters for the mapper.
     */
    void initialize_mapper(MapperParams params);

    /**
     * @brief Set the mapper parameters.
     * 
     * @param params Parameters for the mapper.
     */
    void set_mapper_params(const MapperParams& params) { mapper_params_ = params; }

    /**
     * @brief Get the mapper parameters.
     * 
     * @return const MapperParams& Reference to the mapper parameters.
     */
    const MapperParams& get_mapper_params() const { return mapper_params_; }

    /**
     * @brief Get the voxel mapper.
     * 
     * @return VoxelMapping& Reference to the voxel mapper.
     */
    VoxelMapping& get_mapper() { return mapper_; }

    /**
     * @brief Set the transformation from map to odom frame.
     * 
     * @param map_to_odom_tf Transformation matrix from map to odom frame.
     */
    void set_map_to_odom_tf(const Eigen::Matrix4f& map_to_odom_tf) {
        map_to_odom_tf_ = map_to_odom_tf;
    }

    /**
     * @brief Get the transformation (column major) from map to odom frame.
     * 
     * @return const Eigen::Matrix4f& Transformation matrix from map to odom frame.
     */
    const Eigen::Matrix4f& get_map_to_odom_tf() const { return map_to_odom_tf_; }


    /**
     * @brief Compute the corners of the frustum in the camera frame.
     * 
     * @return std::vector<Eigen::Vector4f> Vector of frustum corners.
     */
    std::vector<Eigen::Vector4f> compute_frustum_corners();

    /**
     * @brief Compute the Axis-Aligned Bounding Box (AABB) of the frustum in the world frame.
     * 
     * @param transform Transformation matrix.
     * @return AABB Axis-Aligned Bounding Box of the frustum.
     */
    AABB compute_frustum_aabb(Eigen::Matrix4f transform);

    /**
     * @brief Get the last computed AABB.
     * 
     * @return const AABB& Reference to the last computed AABB.
     */
    const AABB& get_last_aabb() const {return last_aabb_;};

    /**
     * @brief Get the indices of the AABB.
     * 
     * @param aabb Axis-Aligned Bounding Box.
     * @return Eigen::VectorXi Vector of indices min_x, max_x, min_y, max_y, min_z, max_z.
     */
    Eigen::VectorXi get_aabb_indices(const AABB& aabb);

    /**
     * @brief Set the properties of the image.
     * 
     * @param image_properties Properties of the image.
     */
    void set_image_properties(const ImageProperties& image_properties);

    /**
    * @brief Get the properties of the image.
    * 
    * @return const ImageProperties& Reference to the image properties.
    */
    const ImageProperties& get_image_properties() const { return image_properties_; }

    /**
     * @brief Set the altitude from the DVL.
     * 
     * @param altitude Altitude from the DVL.
     */
    void set_dvl_altitude(float altitude) { dvl_altitude_ = altitude; }

    /**
     * @brief Get the altitude from the DVL.
     * 
     * @return float Altitude from the DVL.
     */
    float get_dvl_altitude() const { return dvl_altitude_; }

    /**
     * @brief Set the position of the ORCA(camera) in the map frame.
     * 
     * @param orca_pos_map_frame Position of the ORCA(camera) in the map frame.
     */
    void set_cam_pos_map_frame(const Eigen::Vector3f& orca_pos_map_frame) {
        orca_pos_map_frame_ = orca_pos_map_frame;
    }

    /**
     * @brief Get the position of the ORCA(camera) in the map frame.
     * 
     * @return const Eigen::Vector3f& Position of the ORCA(camera) in the map frame.
     */
    const Eigen::Vector3f& get_cam_pos_map_frame() const { return orca_pos_map_frame_; }

    /**
     * @brief Set the transformation matrix of the camera in the map frame.
     * 
     * @param cam_transform Transformation matrix of the camera in the map frame.
     */
    void set_cam_transform(const Eigen::Matrix4f& cam_transform) {
        cam_transform_ = cam_transform;
    }

    /**
     * @brief Get the transformation matrix of the camera in the map frame.
     * 
     * @return const Eigen::Matrix4f& Transformation matrix of the camera in the map frame.
     */
    const Eigen::Matrix4f& get_cam_transform() const { return cam_transform_; }

    /**
     * @brief Process the incoming depth image.
     * 
     * @param depth_image Depth image.
     */
    void process_depth_image(const float* depth_image);

    /**
     * @brief Get the updated block of the grid.
     * 
     * @return const std::vector<float>& Updated block of the grid Z, Y, X major.
     */
    const std::vector<float>& get_updated_block() const { return updated_block_; }

    void exploration_timer_callback();

    std::function<void(const std::vector<float>&, const Eigen::VectorXi&)> ros_callback_;

    private:
    VoxelMapping mapper_;
    MapperParams mapper_params_;
    Eigen::Matrix4f map_to_odom_tf_;
    Eigen::Matrix4f cam_transform_;
    Eigen::Vector3f orca_pos_map_frame_;
    AABB last_aabb_;
    ImageProperties image_properties_;
    float dvl_altitude_;
    std::vector<float> updated_block_;
};

#endif // EXPLORATION_MANAGER_HPP


