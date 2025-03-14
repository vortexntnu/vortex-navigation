#include "wayfinder/waypointFinder.h"


double distance(const Point &p1, const Point &p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double WaypointFinder::tileUtility(const double value, const double distance){
    return value/pow(distance, params.localBias);
}

void WaypointFinder::initGaussian(){
    for (int x = 0; x < values.rows(); x++){
        for (int y = 0; y < values.cols(); y++){
            values(x, y) = exp(-1*(
                pow(x-params.centerX, 2)/(2*pow(params.sigmaX, 2)) + 
                pow(y-params.centerY, 2)/(2*pow(params.sigmaY, 2))));
        }
    }
}

WaypointFinder::WaypointFinder(const Eigen::MatrixXd &fullGrid, const double &orcaDepth, const Params &newParams){
    params = newParams;
    values = Eigen::MatrixXd::Zero(fullGrid.rows(), fullGrid.cols());
    obstacles = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(fullGrid.rows(), fullGrid.cols());
    
    updateGrid(fullGrid, orcaDepth, {0, 0});
    
    //update grid fixes obstacles, but also seen points, which we want to start with none, so we need to init gaussian after
    initGaussian();
}

void WaypointFinder::updateGrid(const Eigen::MatrixXd &subGrid, const double &orcaDepth, const Eigen::VectorXi &gridOffset){
    //sets seen points and obstacles
    Point offset = {gridOffset[0], gridOffset[1]};

    assert(subGrid.rows() + offset.x <= values.rows() && subGrid.cols() + offset.y <= values.cols());

    for (int x = 0; x < subGrid.rows(); x++){
        for (int y = 0; y < subGrid.cols(); y++){
            values(x+offset.x, y+offset.y) = 0;
            if (subGrid(x, y) > orcaDepth+params.depthMargin){
                obstacles(x+offset.x, y+offset.y) = true;
            }
            else{
                obstacles(x+offset.x, y+offset.y) = false;
            }
        }
    }
}

Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> dilateMask(const Eigen::MatrixXd &inputMask, int dilationSize) {
    //this is quite stupid, but I don't really want to make the mask a cv::Mat. maybe later
    //covnert to cv::Mat
    cv::Mat mask(inputMask.rows(), inputMask.cols(), CV_8U);
    for (int i = 0; i < inputMask.rows(); ++i) {
        for (int j = 0; j < inputMask.cols(); ++j) {
            mask.at<uchar>(i, j) = inputMask(i, j) ? 255 : 0;
        }
    }

    // Perform the dilation
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1), cv::Point(dilationSize, dilationSize));

    cv::Mat dilatedMask;
    cv::dilate(mask, dilatedMask, element);

    // convert the dilated mask back to a vector
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> outputMask;
    outputMask = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(inputMask.rows(), inputMask.cols());

    for (int i = 0; i < dilatedMask.rows; ++i) {
        for (int j = 0; j < dilatedMask.cols; ++j) {
            outputMask(i, j) = dilatedMask.at<uchar>(i, j) > 0;
        }
    }
    return outputMask;
}

Point WaypointFinder::getWaypoint(){
    Point maxPoint = {params.centerX, params.centerY};
    double maxUtility = 0;

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstaclesWithBuffer;
    obstaclesWithBuffer = dilateMask(obstacles, params.obstaclesMargin);

    for (int x = 0; x < values.rows(); x++){
        for (int y = 0; y < values.cols(); y++){
            if (obstaclesWithBuffer(x, y)){
                continue;
            }
            double utility = tileUtility(values(x, y), distance({x, y}, {params.centerX, params.centerY}));
            if (utility > maxUtility){
                maxUtility = utility;
                maxPoint = {x, y};
            }
        }
    }
    return maxPoint;
}