#include "waypointFinder.hpp"


double distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2){
    return sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
}

double WaypointFinder::tileUtility(const double value, const double distance){
    return value/pow(distance, params.localBias);
}

void WaypointFinder::initGaussian(){
    for (int x = 0; x < values.cols(); x++){
        for (int y = 0; y < values.rows(); y++){
            values(x, y) = exp(-1*(
                pow(x-params.centerX, 2)/(2*pow(params.sigmaX, 2)) + 
                pow(y-params.centerY, 2)/(2*pow(params.sigmaY, 2))));
        }
    }
}

WaypointFinder::WaypointFinder(const Eigen::Vector2i gridSize, const Params &newParams){
    params = newParams;
    values = Eigen::MatrixXd::Zero(gridSize(0), gridSize(1));
    obstacles = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(gridSize(0), gridSize(1));
    unreachableMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(gridSize(0), gridSize(1));
    
    initGaussian();
}

void WaypointFinder::updateGrid(Eigen::MatrixXd &subGrid, const Eigen::Vector3f &dronePosition, const Eigen::VectorXi &aabb){
    //sets seen points and obstacles

    //joergen sends me a row major, so lets transpose
    subGrid.transposeInPlace();

    Point offset = {aabb(0), aabb(2)};

    for (int x = 0; x < subGrid.cols(); x++){
        for (int y = 0; y < subGrid.rows(); y++){
            if (subGrid(x, y) == 1){
                obstacles(x+offset.x, y+offset.y) = true;
            }
            else if (subGrid(x, y) == 0){
                obstacles(x+offset.x, y+offset.y) = false;
            }
        }
    }

    //flag seen points
    for (int coordX = (dronePosition(0) - params.searchRadius)/params.gridSize; coordX < (dronePosition(0) + params.searchRadius)/params.gridSize; coordX++){
        for (int coordY = (dronePosition(1) - params.searchRadius)/params.gridSize; coordY < (dronePosition(1) + params.searchRadius)/params.gridSize; coordY++){
            
            //transform based on grid scale difference
            int x = coordX*params.gridSize;
            int y = coordY*params.gridSize;

            if (coordX < 0|| coordX > values.cols() || coordY < 0 && coordY > values.rows()){
                continue;
            }

            if (distance(Eigen::Vector2d{x, y}, dronePosition) < params.searchRadius){
                values(coordX, coordY) = 0;
            }
        }
    }

    //transposes back
    subGrid.transposeInPlace();

    //find new waypoint
    if (distance(waypoint, dronePosition.head(2)) < params.gridSize){
        findWaypoint(dronePosition);
    }

    //since the obstacle map is updated, we need to recheck if waypoints are reachable
    unreachableMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(values.cols(), values.rows());
}


Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> dilateMask(const Eigen::MatrixXd &inputMask, int dilationSize) {
    //this is quite stupid, but I don't really want to make the mask a cv::Mat. maybe later
    //covnert to cv::Mat
    cv::Mat mask(inputMask.cols(), inputMask.rows(), CV_8U);
    for (int i = 0; i < inputMask.cols(); ++i) {
        for (int j = 0; j < inputMask.rows(); ++j) {
            mask.at<uchar>(i, j) = inputMask(i, j) ? 255 : 0;
        }
    }

    // Perform the dilation
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1), cv::Point(dilationSize, dilationSize));

    cv::Mat dilatedMask;
    cv::dilate(mask, dilatedMask, element);

    // convert the dilated mask back to a vector
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> outputMask;
    outputMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(inputMask.cols(), inputMask.rows());

    for (int i = 0; i < dilatedMask.cols; ++i) {
        for (int j = 0; j < dilatedMask.rows; ++j) {
            outputMask(i, j) = dilatedMask.at<uchar>(i, j) > 0;
        }
    }
    return outputMask;
}

void WaypointFinder::findWaypoint(const Eigen::Vector3f &dronePosition){
    Point maxPoint = {params.centerX, params.centerY};
    double maxUtility = 0;

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstaclesWithBuffer;
    obstaclesWithBuffer = dilateMask(obstacles, params.obstaclesMargin);

    for (int x = 0; x < values.cols(); x++){
        for (int y = 0; y < values.rows(); y++){
            if (obstaclesWithBuffer(x, y) || unreachableMask(x, y)){
                continue;
            }
            double utility = tileUtility(values(x, y), distance(Eigen::Vector2d{x*params.gridSize, y*params.gridSize}, dronePosition));
            if (utility > maxUtility){
                maxUtility = utility;
                maxPoint = {x, y};
            }
        }
    }
    waypoint = {maxPoint.x, maxPoint.y};
}

void WaypointFinder::waypointUnreachable(const Eigen::Vector3f &dronePosition){
    int topX = std::min(waypoint(0) + params.unreachableBlacklist, static_cast<int>(values.cols()));
    int topY = std::min(waypoint(1) + params.unreachableBlacklist, static_cast<int>(values.rows()));
    int bottomX = std::max(waypoint(0) - params.unreachableBlacklist, 0);
    int bottomY = std::max(waypoint(1) - params.unreachableBlacklist, 0);
    
    unreachableMask.block(bottomX, bottomY, topX, topY)
        = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Ones(topX - bottomX, topY - bottomY);

    findWaypoint(dronePosition);
}