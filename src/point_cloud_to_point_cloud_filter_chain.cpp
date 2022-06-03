#include "ros2-filters/point_cloud_to_point_cloud_filter_chain.hpp"

// Component

namespace point_cloud_filters
{
    PointCloudToPointCloudFilterChain::PointCloudToPointCloudFilterChain(rclcpp::Node::SharedPtr nh)
    :   nh_(nh)
    ,   filter_(pclPointcloudStr)
    {
    }

    PointCloudToPointCloudFilterChain::~PointCloudToPointCloudFilterChain()
    {
    }

    void PointCloudToPointCloudFilterChain::onInit()
    {
        #if BUILDING_NODELET
        private_nh_ = getPrivateNodeHandle();
        relative_nh_ = getNodeHandle();
        #else
        private_nh_ = ros::NodeHandle("~");
        relative_nh_ = ros::NodeHandle();
        #endif

        loadROSParams();

        // Initialize point cloud
        filtered_cloud_ = boost::make_shared<pclPointcloud>();
        raw_cloud_ = boost::make_shared<pclPointcloud>();

        sub_ = relative_nh_.subscribe<sensor_msgs::PointCloud2>(input_pc_topic_, 1, [this](const sensor_msgs::PointCloud2::ConstPtr & msg){this->pointcloudCB(msg);});
        pub_ = relative_nh_.advertise<sensor_msgs::PointCloud2>(output_pc_topic_, 1);

        try
        {
            if(!filter_.configure(point_cloud_filter_chain, private_nh_))
            {
                ROS_ERROR_STREAM(
                    ros::this_node::getName()
                    << " Failed to start point cloud to point cloud filter chain node."
                );
                return;
            }
        }
        catch(const pluginlib::PluginlibException & e)
        {
            ROS_ERROR_STREAM(
                ros::this_node::getName()
                << " "
                << __func__
                << " caught an error: "
                << e.what();
            );
            return;
        }
        catch(...)
        {
            ROS_ERROR_STREAM(
                ros::this_node::getName()
                << " "
                << __func__
                << " caught an error!"
            );
            return;
        }

        ROS_INFO_STREAM(
            ros::this_node::getName()
            << " Started point cloud to point cloud filtering node."
        );
    }

    void PointCloudToPointCloudFilterChain::loadROSParams()
    {
        private_nh_.param<std::string>("input_pointcloud_topic", input_pc_topic_, "input_cloud");
        private_nh_.param<std::string>("output_pointcloud_topic", output_pc_topic_, "output_cloud");
    }

    void  PointCloudToPointCloudFilterChain::pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
        // Convert to pcl point cloud
        // raw_cloud_->clear();
        raw_cloud_ = boost::make_shared<point_cloud_filters::pclPointcloud>();
        pcl::fromROSMsg(*msg, *raw_cloud_);

        // Apply point cloud filters
        if(filter_.update(raw_cloud_, filtered_cloud_))
        {
            // Publish filtered point cloud only if successful
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*filtered_cloud_, msg);
            pub_.publish(msg);
            filtered_cloud_->clear();
        }
        else
        {
            ROS_ERROR_STREAM_THROTTLE(
                1,
                ros::this_node::getName()
                << " "
                << __func__
                << ": Filtering the point cloud from time "
                << msg->header.stamp.sec
                << "."
                << msg->header.stamp.nsec
                << " has failed."
            );
        }
    }
} // namespace point_cloud_filters