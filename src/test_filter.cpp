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

        // Setup dynamic parameters server
        dyn_params_handler_ = params_interface_->add_on_set_parameters_callback([this](std::vector<rclcpp::Parameter> parameters){return this->dynamicParametersCallback(parameters);});

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

    rcl_interfaces::msg::SetParametersResult TestFilter::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        const auto & filter_name = filters::FilterBase<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::getName();
        bool unknown_param = false;

        try
        {
            // Update parameters
            for (auto parameter : parameters)
            {
                const auto & type = parameter.get_type();
                const auto & name = parameter.get_name();

                // COMMON START
                switch (type)
                {
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    if (name ==  filter_name + "filter_limit_min")
                        this->filter_limit_min_ = parameter.as_double();
                    else if (name ==  filter_name + "filter_limit_max")
                        this->filter_limit_max_ = parameter.as_double();
                    // LOCAL PARAM START
                    else if (name == filter_name + "double")
                        double_= parameter.as_double();
                    // LOCAL PARAM END
                    else
                        unknown_param = true;
                    break;

                case rclcpp::ParameterType::PARAMETER_STRING:
                    if (name ==  filter_name + "filter_field_name")
                        this->filter_field_name_ = parameter.as_string();
                    else if (name ==  filter_name + "input_frame")
                        this->tf_input_frame_ = parameter.as_string();
                    else if (name ==  filter_name + "output_frame")
                        this->tf_output_frame_ = parameter.as_string();
                    // LOCAL PARAM START
                    else if (name == filter_name + "string")
                        string_= parameter.as_string();
                    // LOCAL PARAM END
                    else
                        unknown_param = true;
                    break;

                case rclcpp::ParameterType::PARAMETER_BOOL:
                    if (name ==  filter_name + "filter_limit_negative")
                        this->filter_limit_negative_ = parameter.as_bool();
                    else if (name ==  filter_name + "keep_organized")
                        this->keep_organized_ = parameter.as_bool();
                    // LOCAL PARAM START
                    else if (name == filter_name + "debug")
                        debug_ = parameter.as_bool();
                    // LOCAL PARAM END
                    else
                        unknown_param = true;
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    // LOCAL PARAM START
                    if ( name == filter_name + "int")
                        int_= parameter.as_int();
                    // LOCAL PARAM END
                    else
                        unknown_param = true;
                    break;

                default:
                    unknown_param = true;
                    break;
                }
                // COMMON END

                if (unknown_param)
                    RCLCPP_WARN_STREAM(
                        logging_interface_->get_logger(),
                        "("
                        << __func__
                        << ") unknown param - "
                        << name
                        << " was not set with "
                        << parameter.value_to_string()
                    );
                else
                    BaseFilter<std::shared_ptr<sensor_msgs::msg::PointCloud2>>::printParamUpdate(name.c_str(), parameter.value_to_string());

            }
        }
        catch(const rclcpp::ParameterTypeException & e)
        {
            RCLCPP_ERROR_STREAM(
                this->logging_interface_->get_logger(),
                "("
                << __func__
                << ") caught an error - "
                << e.what()
            );

            result.successful = false;
            result.reason = "ParameterTypeException";
            return result;
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(
                this->logging_interface_->get_logger(),
                "("
                << __func__
                << ") caught an error."
            );

            result.successful = false;
            result.reason = "an error caught";
            return result;
        }

        if (unknown_param)
        {
            result.successful = false;
            result.reason = "There was an unkonwn param";
        }
        else
        {
            result.successful = true;
            result.reason = "success";
        }

        return result;
    }

} // namespace point_cloud_filters