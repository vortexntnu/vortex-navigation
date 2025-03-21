#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <Eigen/Dense> // Include the Eigen library

struct Params{
    double sigmaX; //paramters for the 2D gaussian
    double sigmaY;
    int centerX;
    int centerY;
    double localBias; //how much will the drone prefer to explore close to current location
    int obstaclesMargin; //how many cells around an obstacle are considered untouchable
    double searchRadius; //radius of what is considered seen around the drone (meters)
    double gridSize = 0.1; //size of each grid cell in meters
};

struct Point{
    int x;
    int y;
};


double distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);

class WaypointFinder {
    Params params;

    Eigen::MatrixXd values;    
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacles; // Eigen matrix of bools

    void initGaussian();
    double tileUtility(const double value, const double distance);

    public:
        WaypointFinder(){};
        WaypointFinder(const Eigen::Vector2i gridSize, const Params &newParams);

        //subgrid contains data around the drone, 0 is free, 1 is obstacle, -1 is unknown
        //dronePosition is the current drone position in meters
        void updateGrid(Eigen::MatrixXd &subGrid, const Eigen::Vector2d dronePosition, const Eigen::VectorXi &aabb);

        Point getWaypoint(const Eigen::Vector2d dronePosition);

};
