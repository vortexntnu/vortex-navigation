#include "wayfinder/waypointFinder.h"

#include <cmath>
#include <assert>

double distance(const Point &p1, const Point &p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double WaypointFinder::tileUtility(const double value, const double distance){
    return value/pow(distance, params.localBias);
}

void WaypointFinder::initGaussian(){
    for (int x = 0; x < values.size(); x++){
        for (int y = 0; y < values[0].size(); y++){
            values[x][y] = exp(-1*(
                pow(x-params.centerX, 2)/(2*pow(params.sigmaX, 2)) + 
                pow(y-params.centerY, 2)/(2*pow(params.sigmaY, 2))));
        }
    }
}

WaypointFinder::WaypointFinder(const std::vector<std::vector<double>> &fullGrid, const double &orcaDepth, const Params &newParams){
    params = newParams;
    values = std::vector<std::vector<double>>(fullGrid.size(), std::vector<double>(fullGrid[0].size(), 0));
    walls = std::vector<std::vector<bool>>(fullGrid.size(), std::vector<bool>(fullGrid[0].size(), false));
    initGaussian();

    //TODO also, update the walls and add seen points
    
}

void WaypointFinder::updateGrid(const std::vector<std::vector<double>> &subGrid, const double &orcaDepth){
    assert(subGrid.size() <= values.size() && subGrid[0].size() <= values.size());
    for (int x = 0; x < subGrid.size(); x++){
        for (int y = 0; y < subGrid[0].size(); y++){
            values[x][y] = 0;
            if (subGrid[x][y] > orcaDepth+params.depthMargin){
                walls[x][y] = true;
            }
            else{
                walls[x][y] = false;
            }
        }
    }
}

void dilateMask(const std::vector<std::vector<bool>> &inputMask, std::vector<std::vector<bool>> &outputMask, int dilationSize) {
    //this is quite stupid, but I don't really want to make the mask a cv::Mat. maybe later
    //covnert to cv::Mat
    cv::Mat mask(inputMask.size(), inputMask[0].size(), CV_8U);
    for (int i = 0; i < inputMask.size(); ++i) {
        for (int j = 0; j < inputMask[0].size(); ++j) {
            mask.at<uchar>(i, j) = inputMask[i][j] ? 255 : 0;
        }
    }

    // Perform the dilation
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1), cv::Point(dilationSize, dilationSize));

    cv::Mat dilatedMask;
    cv::dilate(mask, dilatedMask, element);

    // convert the dilated mask back to a vector
    outputMask.resize(dilatedMask.rows, std::vector<bool>(dilatedMask.cols, false));
    for (int i = 0; i < dilatedMask.rows; ++i) {
        for (int j = 0; j < dilatedMask.cols; ++j) {
            outputMask[i][j] = dilatedMask.at<uchar>(i, j) > 0;
        }
    }
}

Point WaypointFinder::getWaypoint(){
    Point maxPoint = {params.centerX, params.centerY};
    double maxUtility = 0;

    vector<vector<bool>> wallsWithBuffer;
    dilateMask(wallsGrid, wallsWithBuffer, params.wallsMargin);

    for (int x = 0; x < values.size(); x++){
        for (int y = 0; y < values[0].size(); y++){
            if (walls[x][y]){
                continue;
            }
            double utility = tileUtility(values[x][y], distance({x, y}, {params.centerX, params.centerY}));
            if (utility > maxUtility){
                maxUtility = utility;
                maxPoint = {x, y};
            }
        }
    }
    return maxPoint;
}