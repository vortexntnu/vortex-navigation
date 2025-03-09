#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "grid.h"

class WayfinderNode : public rclcpp::Node {
    public:
        WayfinderNode(const rclcpp::NodeOptions& options);
        ~WayfinderNode() {};
    private:
        //rclcpp::Subscription<type>::SharedPtr pointSub;
        //rclcpp::Publisher<type>::SharedPtr wayPointPub;

        Grid grid;

        //void pointCallback(const type msg);
        void positionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
};

