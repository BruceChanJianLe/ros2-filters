#include "ros2-filters/point_cloud_to_point_cloud_filter_chain.hpp"

// Component

namespace point_cloud_filters
{
    PointCloudToPointCloudFilterChain::PointCloudToPointCloudFilterChain(rclcpp::Node::SharedPtr nh)
    :   nh_(nh)
    ,   filter_(filter_chain_type)
    {
    }

    PointCloudToPointCloudFilterChain::~PointCloudToPointCloudFilterChain()
    {
        unloadROSParams();
    }

    void PointCloudToPointCloudFilterChain::onInit()
    {
        loadROSParams();

        // Initialize publisher
        pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(output_pc_topic_, 5);

        // Initialize subscriber
        mf_sub_.subscribe(nh_, input_pc_topic_, rmw_qos_profile_sensor_data);
        mf_sub_.registerCallback([this](const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & msg){this->pointcloudCB(msg);});

        try
        {
            // Configure filter chain
            if(!filter_.configure("", nh_->get_node_logging_interface(), nh_->get_node_parameters_interface()))
            {
                RCLCPP_ERROR_STREAM(
                    nh_->get_logger(),
                    nh_->get_name()
                    << " Failed to start point cloud to point cloud filter chain node."
                );
                return;
            }
        }
        catch(const pluginlib::PluginlibException & e)
        {
            RCLCPP_ERROR_STREAM(
                nh_->get_logger(),
                nh_->get_name()
                << " "
                << __func__
                << " caught an error: "
                << e.what();
            );
            return;
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(
                nh_->get_logger(),
                nh_->get_name()
                << " "
                << __func__
                << " caught an error!"
            );
            return;
        }

        RCLCPP_INFO_STREAM(
            nh_->get_logger(),
            nh_->get_name()
            << " Started point cloud to point cloud filtering node."
        );
    }

    void PointCloudToPointCloudFilterChain::loadROSParams()
    {
        // Declare params
        nh_->declare_parameter<std::string>("input_pointcloud_topic", "input_cloud");
        nh_->declare_parameter<std::string>("output_pointcloud_topic", "output_cloud");

        // Get params
        try
        {
            if(nh_->get_parameter("input_pointcloud_topic", input_pc_topic_))
                RCLCPP_INFO_STREAM(
                    nh_->get_logger(),
                    "successfully loaded value: "
                    << input_pc_topic_
                );
            if(nh_->get_parameter("output_pointcloud_topic", output_pc_topic_))
                RCLCPP_INFO_STREAM(
                    nh_->get_logger(),
                    "successfully loaded value: "
                    << output_pc_topic_
                );
        }
        catch(const rclcpp::ParameterTypeException & e)
        {
            RCLCPP_ERROR_STREAM(
                nh_->get_logger(),
                nh_->get_name()
                << " "
                << __func__
                << ": caught an error - "
                << e.what()
            );
        }
    }

    void PointCloudToPointCloudFilterChain::unloadROSParams()
    {
        // Undeclare params
        nh_->undeclare_parameter("input_pointcloud_topic");
        nh_->undeclare_parameter("output_pointcloud_topic");
    }


    void  PointCloudToPointCloudFilterChain::pointcloudCB(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & msg)
    {
        // Initialize filtered cloud
        raw_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        filtered_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        *raw_cloud_ = *msg;

        // Apply point cloud filters
        if(filter_.update(raw_cloud_, filtered_cloud_))
        {
            // Publish filtered point cloud only if successful
            pub_->publish(std::move(*filtered_cloud_));
        }
        else
        {
            RCLCPP_ERROR_STREAM_THROTTLE(
                nh_->get_logger(),
                *nh_->get_clock(),
                1000,
                nh_->get_name()
                << " "
                << __func__
                << ": Filtering the point cloud from time "
                << msg->header.stamp.sec
                << "."
                << msg->header.stamp.nanosec
                << " has failed."
            );
        }
    }
} // namespace point_cloud_filters