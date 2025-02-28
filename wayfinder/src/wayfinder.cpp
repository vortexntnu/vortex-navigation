#include "wayfinder.h"

WayfinderNode::WayfinderNode(const rclcpp::NodeOptions& options) : Node("wayfinderNode", options){
    //pointSub = this->create_subscription<type>("point_topic", 10, std::bind(&WayfinderNode::pointCallback, this, std::placeholders::_1));
    //wayPointPub = this->create_publisher<type>("waypoint_topic", 10);
    grid = Grid();
}

void WayfinderNode::pointCallback(const type msg){
    //extract the points from msg
    std::vector<Point> wallPoints;
    grid.updateGrid(wallPoints);
}