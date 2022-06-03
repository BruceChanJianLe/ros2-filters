#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "filters/filter_chain.h"
#include "message_filters/subscriber.h"

// STL
#include <string>
#include <memory>
#include <mutex>

namespace point_cloud_filters
{
    /// \brief Filter chain parameter name
    const std::string point_cloud_filter_chain = "point_cloud_filter_chain";
    const std::string filter_chain_type = "sensor_msgs::msg::PointCloud2";

    class PointCloudToPointCloudFilterChain
    #if BUILDING_NODELET
    :   public nodelet::Nodelet
    #endif
    {
    public:
        /// \brief Construct a new Point Cloud To Point Cloud Filter Chain object
        explicit PointCloudToPointCloudFilterChain(rclcpp::Node::SharedPtr nh);

        /// \brief Destroy the Point Cloud To Point Cloud Filter Chain object
        ~PointCloudToPointCloudFilterChain();

        /// \brief Initialize and start point cloud filtering node
        virtual void onInit();

    protected:
        /// \brief Nodehandler
        rclcpp::Node::SharedPtr nh_;

        /// \brief Filter Chain
        filters::FilterChain<std::shared_ptr<sensor_msgs::msg::PointCloud2>> filter_;

        /// \brief Filtered Point Cloud Publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2> pub_;

        /// \brief Message Filter for Raw Point Cloud Subscriber
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> mf_sub_;

        /// \brief Filtered Point Cloud
        sensor_msgs::msg::PointCloud2::SharedPtr filtered_cloud_;

        /// \brief Raw Point Cloud
        std::shared_ptr<sensor_msgs::msg::PointCloud2> raw_cloud_;

        /// \brief Mutex to lock from callback and filtering
        std::recursive_mutex m_;

    private:
        /// \brief Load parameters from ROS Server
        void loadROSParams();

        /// \brief
        void pointcloudCB(const sensor_msgs::msg::PointCloud2::ConstPtr & msg);

        /// \brief Input pointcloud topic
        std::string input_pc_topic_;

        /// \brief Output pointcloud topic
        std::string output_pc_topic_;
    };

} // namespace point_cloud_filters
