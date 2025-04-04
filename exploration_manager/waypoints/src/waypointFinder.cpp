#include "waypointFinder.hpp"

double distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2){
    return sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
}

double WaypointFinder::tileUtility(const double value, const double distance){
    return value / pow(distance, params.localBias);
}

void WaypointFinder::initGaussian(){
    for (int y = 0; y < values.rows(); y++){
        for (int x = 0; x < values.cols(); x++){
            values(y, x) = exp(-1 * (
                pow(x - params.centerX, 2) / (2 * pow(params.sigmaX, 2)) + 
                pow(y - params.centerY, 2) / (2 * pow(params.sigmaY, 2))));
        }
    }
}

WaypointFinder::WaypointFinder(const Eigen::Vector2i gridSize, const WaypointParams &newParams){
    params = newParams;
    values = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(gridSize(1), gridSize(0));
    obstacles = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(gridSize(1), gridSize(0));
    unreachableMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(gridSize(1), gridSize(0));
    
    initGaussian();
}

void WaypointFinder::updateGrid(Eigen::MatrixXf &subGrid, const Eigen::Vector3f &dronePosition, const Eigen::VectorXi &aabb){
    Point offset = {aabb(0), aabb(2)};

    for (int y = 0; y < subGrid.rows(); y++){
        for (int x = 0; x < subGrid.cols(); x++){
            if (subGrid(y, x) == 1){
                obstacles(y + offset.y, x + offset.x) = true;
            }
            else if (subGrid(y, x) == 0){
                obstacles(y + offset.y, x + offset.x) = false;
            }
        }
    }

    for (int coordY = (dronePosition(1) - params.searchRadius) / params.resolution; coordY < (dronePosition(1) + params.searchRadius) / params.resolution; coordY++){
        for (int coordX = (dronePosition(0) - params.searchRadius) / params.resolution; coordX < (dronePosition(0) + params.searchRadius) / params.resolution; coordX++){
            int x = coordX * params.resolution;
            int y = coordY * params.resolution;

            if (coordX < 0 || coordX >= values.cols() || coordY < 0 || coordY >= values.rows()){
                continue;
            }

            if (distance(Eigen::Vector2d{x, y}, dronePosition.head(2).cast<double>()) < params.searchRadius){
                values(coordY, coordX) = 0;
            }
        }
    }

    if (distance(waypoint_.cast<double>(), dronePosition.head(2).cast<double>()) < params.resolution || !waypointSet){
        findWaypoint(dronePosition);
    }

    unreachableMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(values.rows(), values.cols());
}

Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dilateMask(const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &inputMask, int dilationSize) {
    cv::Mat mask(inputMask.rows(), inputMask.cols(), CV_8U);
    for (int y = 0; y < inputMask.rows(); ++y) {
        for (int x = 0; x < inputMask.cols(); ++x) {
            mask.at<uchar>(y, x) = inputMask(y, x) ? 255 : 0;
        }
    }

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1), cv::Point(dilationSize, dilationSize));
    cv::Mat dilatedMask;
    cv::dilate(mask, dilatedMask, element);

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> outputMask;
    outputMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(inputMask.rows(), inputMask.cols());

    for (int y = 0; y < dilatedMask.rows; ++y) {
        for (int x = 0; x < dilatedMask.cols; ++x) {
            outputMask(y, x) = dilatedMask.at<uchar>(y, x) > 0;
        }
    }
    return outputMask;
}

void WaypointFinder::findWaypoint(const Eigen::Vector3f &dronePosition){
    Point maxPoint = {params.centerX, params.centerY};
    double maxUtility = 0;

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> obstaclesWithBuffer;
    obstaclesWithBuffer = dilateMask(obstacles, params.obstaclesMargin);

    for (int y = 0; y < values.rows(); y++){
        for (int x = 0; x < values.cols(); x++){
            if (obstaclesWithBuffer(y, x) || unreachableMask(y, x)){
                continue;
            }
            double utility = tileUtility(values(y, x), distance(Eigen::Vector2d{x * params.resolution, y * params.resolution}, dronePosition.head(2).cast<double>()));
            if (utility > maxUtility){
                maxUtility = utility;
                maxPoint = {x, y};
            }
        }
    }
    waypointSet = true;
    waypoint_ = {maxPoint.x, maxPoint.y};

    std::cout << "Waypoint: " << waypoint_(0) << ", " << waypoint_(1) << std::endl;
}

void WaypointFinder::waypointUnreachable(const Eigen::Vector3f &dronePosition){
    int topX = std::min(waypoint_(0) + params.unreachableBlacklist, static_cast<int>(values.cols()));
    int topY = std::min(waypoint_(1) + params.unreachableBlacklist, static_cast<int>(values.rows()));
    int bottomX = std::max(waypoint_(0) - params.unreachableBlacklist, 0);
    int bottomY = std::max(waypoint_(1) - params.unreachableBlacklist, 0);
    
    unreachableMask.block(bottomY, bottomX, topY - bottomY, topX - bottomX)
        = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Ones(topY - bottomY, topX - bottomX);

    findWaypoint(dronePosition);
}

bool WaypointFinder::getWaypoint(Eigen::Vector2f &waypoint){
    if (!waypointSet){
        return false;
    }
    waypoint(0) = waypoint_(0);
    waypoint(1) = waypoint_(1);
    return true;
};
