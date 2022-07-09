#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ros2-filters/base_filter.hpp"

// STL
#include <string>
#include <memory>

namespace point_cloud_filters
{
    class TestFilter : public BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>
    {
    public:
        /** \brief Construct a new Test Filter object */
        TestFilter();

        /** \brief Destroy the Test Filter object */
        virtual ~TestFilter();

        /** \brief Configure filter
          * \return true if configure successful
          * \return false if failed configure
          */
        bool configure() override;

        /** \brief Apply filter to input point cloud
          * \param input_pointcloud (boost) shared pointer to input point cloud data
          * \param output_pointcloud (boost) shared pointer to output point cloud result
          * \return true if filtered point cloud succesfully
          * \return false if failed to filter point cloud
          */
        bool update(const std::shared_ptr<sensor_msgs::msg::PointCloud2> & input_pointcloud, std::shared_ptr<sensor_msgs::msg::PointCloud2> & output_pointcloud) override;

      private:
        std::string string_;
        double double_;
        int int_;
        bool debug_;
    };

} // namespace point_cloud_filters
