#include "ros2-filters/point_cloud_to_point_cloud_filter_chain.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("point_cloud_filters_node");

    point_cloud_filters::PointCloudToPointCloudFilterChain node(nh);

    // Initialize and start point cloud filtering node
    node.onInit();

    // Spin node
    rclcpp::WallRate rate(200);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(nh);
        rate.sleep();
    }

    return 0;
}