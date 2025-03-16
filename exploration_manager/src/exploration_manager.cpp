#include <exploration_manager/exploration_manager.hpp>

ExplorationManager::ExplorationManager(/* args */) {
}

ExplorationManager::~ExplorationManager() {
    mapper_.~VoxelMapping();
}

void ExplorationManager::initialize_mapper(MapperParams params) {
    mapper_ = VoxelMapping(params.resolution, params.size_x, params.size_y, params.size_z, params.min_depth, params.max_depth, params.log_odds_occupied, params.log_odds_free, params.log_odds_min, params.log_odds_max);
    set_mapper_params(params);
}

void ExplorationManager::set_image_properties(const ImageProperties& image_properties) {
    image_properties_ = image_properties;
    mapper_.set_K(image_properties.fx, image_properties.fy, image_properties.cx, image_properties.cy);
    mapper_.set_image_size(image_properties.width, image_properties.height);
}

std::vector<Eigen::Vector4f> ExplorationManager::compute_frustum_corners() {
    ImageProperties image_properties = get_image_properties();

    int w = image_properties.width;
    int h = image_properties.height;
    float near_plane = image_properties.min_depth;
    float far_plane = image_properties.max_depth;
    float fx_ = image_properties.fx;
    float fy_ = image_properties.fy;
    float cx_ = image_properties.cx;
    float cy_ = image_properties.cy;

    std::vector<Eigen::Vector4f> frustum_corners = {
        // Near plane corners (TL, TR, BR, BL)
        Eigen::Vector4f((0 - cx_) * near_plane / fx_, (0 - cy_) * near_plane / fy_, near_plane, 1.0f),
        Eigen::Vector4f((w - cx_) * near_plane / fx_, (0 - cy_) * near_plane / fy_, near_plane, 1.0f),
        Eigen::Vector4f((w - cx_) * near_plane / fx_, (h - cy_) * near_plane / fy_, near_plane, 1.0f),
        Eigen::Vector4f((0 - cx_) * near_plane / fx_, (h - cy_) * near_plane / fy_, near_plane, 1.0f),
        // Far plane corners (TL, TR, BR, BL)
        Eigen::Vector4f((0 - cx_) * far_plane / fx_, (0 - cy_) * far_plane / fy_, far_plane, 1.0f),
        Eigen::Vector4f((w - cx_) * far_plane / fx_, (0 - cy_) * far_plane / fy_, far_plane, 1.0f),
        Eigen::Vector4f((w - cx_) * far_plane / fx_, (h - cy_) * far_plane / fy_, far_plane, 1.0f),
        Eigen::Vector4f((0 - cx_) * far_plane / fx_, (h - cy_) * far_plane / fy_, far_plane, 1.0f)
    };

    return frustum_corners;
}

AABB ExplorationManager::compute_frustum_aabb(Eigen::Matrix4f T) {

    Eigen::Vector3f min_corner(std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max());
    Eigen::Vector3f max_corner(std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest());

    std::vector<Eigen::Vector4f> frustum_corners = compute_frustum_corners();

    auto update_corners = [&](const Eigen::Vector4f& corner) {
        Eigen::Vector4f world_point = T * corner;
        Eigen::Vector3f world_xyz = world_point.head<3>() / world_point.w();

        min_corner = min_corner.cwiseMin(world_xyz);
        max_corner = max_corner.cwiseMax(world_xyz);
    };

    std::for_each(frustum_corners.begin(), frustum_corners.end(), update_corners);

    return {min_corner, max_corner};
}

Eigen::VectorXi ExplorationManager::get_aabb_indices(const AABB& aabb) {

    Eigen::VectorXi aabb_indices(6);
    MapperParams params = get_mapper_params();
    float resolution = params.resolution;
    uint size_x = params.size_x;
    uint size_y = params.size_y;
    uint size_z = params.size_z;

    aabb_indices[0] = std::max(0, static_cast<int>(std::floor(aabb.min_corner.x() / resolution)));
    aabb_indices[1] = std::min(static_cast<int>(size_x - 1), static_cast<int>(std::floor(aabb.max_corner.x() / resolution)));
    aabb_indices[2] = std::max(0, static_cast<int>(std::floor(aabb.min_corner.y() / resolution)));
    aabb_indices[3] = std::min(static_cast<int>(size_y - 1), static_cast<int>(std::floor(aabb.max_corner.y() / resolution)));
    aabb_indices[4] = std::max(0, static_cast<int>(std::floor(aabb.min_corner.z() / resolution)));
    aabb_indices[5] = std::min(static_cast<int>(size_z - 1), static_cast<int>(std::floor(aabb.max_corner.z() / resolution)));

    return aabb_indices;
}

void ExplorationManager::process_depth_image(const float* depth_image) {

    Eigen::Matrix4f T = get_cam_transform();
    AABB aabb = compute_frustum_aabb(T);
    last_aabb_ = aabb;
    Eigen::VectorXi aabb_indices = get_aabb_indices(aabb);

    mapper_.integrate_depth(depth_image, T, aabb_indices);

    updated_block_ = mapper_.get_grid_block(aabb_indices);

}

