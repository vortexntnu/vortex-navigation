#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cassert>

struct Params{
    double sigmaX;
    double sigmaY;
    int centerX;
    int centerY;
    double localBias; //how much will the drone prefer to explore close to current location
    int wallsMargin;
    double depthMargin;
};

struct Point{
    int x;
    int y;
};

double distance(const Point &p1, const Point &p2);

class WaypointFinder {
    Params params;
    std::vector<std::vector<double>> values;
    std::vector<std::vector<bool>> walls;

    void initGaussian();
    double tileUtility(const double value, const double distance);


    public:
        WaypointFinder(){};
        WaypointFinder(const std::vector<std::vector<double>> &fullGrid, const double &orcaDepth, const Params &newParams);

        void updateGrid(const std::vector<std::vector<double>> &subGrid, const double &orcaDepth);

        Point getWaypoint();

};
