#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <Eigen/Dense> // Include the Eigen library

struct WaypointParams{
    double sigmaX; //paramters for the 2D gaussian
    double sigmaY;
    int centerX;
    int centerY;
    double localBias; //how much will the drone prefer to explore close to current location
    int obstaclesMargin; //how many cells around an obstacle are considered untouchable
    double searchRadius; //radius of what is considered seen around the drone (meters)
    double resolution = 0.1; //size of each grid cell in meters
    int unreachableBlacklist = 20; //how big a buffer around an unreachable waypoint is considered unreachable
};

struct Point{
    int x;
    int y;
};


double distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);

class WaypointFinder {
    WaypointParams params;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> values;    
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacles; // Eigen matrix of obstacle mask
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> unreachableMask; // unreachable waypoints. resets every time we update the grid

    Eigen::Vector2i waypoint_; //waypoint in grid coordinates
    bool waypointSet = false; //if the waypoint is set or not

    void initGaussian();
    double tileUtility(const double value, const double distance);

    void findWaypoint(const Eigen::Vector3f &dronePosition);

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dilateMask(const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &inputMask, int dilationSize);

    public:
        WaypointFinder(){};
        WaypointFinder(const Eigen::Vector2i gridSize, const WaypointParams &newParams);

        //subgrid contains data around the drone, 0 is free, 1 is obstacle, -1 is unknown
        //dronePosition is the current drone xy position in meters
        void updateGrid(Eigen::MatrixXf &subGrid, const Eigen::Vector3f &dronePosition, const Eigen::VectorXi &aabb);

        void waypointUnreachable(const Eigen::Vector3f &dronePosition);

        bool getWaypoint(Eigen::Vector2f &waypoint);

        const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& getObstacles() const {
            return obstacles;
        }

        const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& getValues() const {
            return values;
        }

};
