#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <Eigen/Dense> // Include the Eigen library
#include <cassert>

struct Params{
    double sigmaX;
    double sigmaY;
    int centerX;
    int centerY;
    double localBias; //how much will the drone prefer to explore close to current location
    int obstaclesMargin;
    double depthMargin;
};

struct Point{
    int x;
    int y;
};

double distance(const Point &p1, const Point &p2);

class WaypointFinder {
    Params params;

    Eigen::MatrixXd values;    
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacles; // Eigen matrix of bools

    void initGaussian();
    double tileUtility(const double value, const double distance);


    public:
        WaypointFinder(){};
        WaypointFinder(const Eigen::MatrixXd &fullGrid, const double &orcaDepth, const Params &newParams);

        void updateGrid(const  Eigen::MatrixXd &subGrid, const double &orcaDepth, const Eigen::VectorXi &gridOffset);

        Point getWaypoint();

};
