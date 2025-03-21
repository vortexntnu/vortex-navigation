#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <Eigen/Dense> // Include the Eigen library

struct Params{
    double sigmaX;
    double sigmaY;
    int centerX;
    int centerY;
    double localBias; //how much will the drone prefer to explore close to current location
    int obstaclesMargin;
    double depthMargin;
    double ignoreDepthMargin;
    double searchRadius; //radius of what is considered seen around the drone
    double gridSize = 0.1; //size of each grid cell in meters
};

struct Point{
    int x;
    int y;
};

struct Point3{
    double x, y, z;
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
        WaypointFinder(const Point gridSize, const Params &newParams);

        //subgrid contains absolute depth values
        void updateGrid(Eigen::MatrixXd &subGrid, const Point dronePosition, const Eigen::VectorXi &aabb);

        Point getWaypoint();

};
