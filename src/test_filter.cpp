#include "ros2-filters/test_filter.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(point_cloud_filters::TestFilter, filters::FilterBase<std::shared_ptr<sensor_msgs::msg::PointCloud2>>)

namespace point_cloud_filters
{
    TestFilter::TestFilter()
    {
    }

    TestFilter::~TestFilter() = default;

    bool TestFilter::configure()
    {
        if(!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("string"), string_))
        {
            RCLCPP_ERROR_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << ": Error, TestFilter was not given string param."
            );
            return false;
        }

        if(!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("double"), double_))
        {
            RCLCPP_ERROR_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << ": Error, TestFilter was not given double param."
            );
            return false;
        }

        if(!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("int"), int_))
        {
            ;
            RCLCPP_ERROR_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << ": Error, TestFilter was not given int param."
            );
            return false;
        }

        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("debug"), debug_))
        {
            RCLCPP_ERROR_STREAM(
                logging_interface_->get_logger(),
                "Median filter did not find parameter debug. Disabling debug output."
            );
            debug_ = false;
        }

        RCLCPP_INFO_STREAM(
            logging_interface_->get_logger(),
            "string: "
            << string_
            << ", double: "
            << double_
            << ", int: "
            << int_
            << ", debug: "
            << debug_
        );

        // COMMON START
        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("filter_field_name"), filter_field_name_))
        {
            RCLCPP_WARN_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << " - Setting default value for filter_field_name."
            );
            filter_field_name_ = "z";
        }

        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("filter_limit_min"), filter_limit_min_))
        {
            RCLCPP_WARN_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << " - Setting default value for filter_limit_min."
            );
            filter_limit_min_ = 0.0;
        }

        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("filter_limit_max"), filter_limit_max_))
        {
            RCLCPP_WARN_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << " - Setting default value for filter_limit_max."
            );
            filter_limit_max_ = 1.0;
        }

        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("filter_limit_negative"), filter_limit_negative_))
        {
            RCLCPP_WARN_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << " - Setting default value for filter_limit_negative."
            );
            filter_limit_negative_ = false;
        }

        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("keep_organized"), keep_organized_))
        {
            RCLCPP_WARN_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << " - Setting default value for keep_organized."
            );
            keep_organized_ = false;
        }

        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("input_frame"), tf_input_frame_))
        {
            RCLCPP_WARN_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << " - Setting default value for input_frame."
            );
            tf_input_frame_ = "";
        }

        if (!BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getParamFilter(std::string("output_frame"), tf_output_frame_))
        {
            RCLCPP_WARN_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << " - Setting default value for output_frame."
            );
            tf_output_frame_ = "";
        }
        // COMMON END

        // Update filter implementation

        return true;
    }

    bool TestFilter::update(const std::shared_ptr<sensor_msgs::msg::PointCloud2> & input_pointcloud, std::shared_ptr<sensor_msgs::msg::PointCloud2> & output_pointcloud)
    {
        output_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        if(!filters::FilterBase<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::configured_)
        {
            RCLCPP_ERROR_STREAM(
                logging_interface_->get_logger(),
                logging_interface_->get_logger_name()
                << " "
                << __func__
                << ": configure your filter before calling update()."
            );
            return false;
        }

        if(debug_)
            RCLCPP_INFO_STREAM(
                logging_interface_->get_logger(),
                "string: "
                << string_
                << ", double: "
                << double_
                << ", int: "
                << int_
                << ", debug: "
                << debug_
            );

        output_pointcloud = std::move(input_pointcloud);

        return true;
    }

} // namespace point_cloud_filters