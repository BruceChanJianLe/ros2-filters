#pragma once

// Filters
#include "filters/filter_base.hpp"

// STL
#include <string>


namespace point_cloud_filters
{
    template <typename T>
    class BaseFilter : public filters::FilterBase<T>
    {
    public:
        BaseFilter() {};
        virtual ~BaseFilter() = 0;

        /** \brief Configure filter
          * \return true if configure successful
          * \return false if failed configure
          */
        virtual bool configure() = 0;

        /** \brief Apply filter to input point cloud
          * \param input_pointcloud (boost) shared pointer to input point cloud data
          * \param output_pointcloud (boost) shared pointer to output point cloud result
          * \return true if filtered point cloud succesfully
          * \return false if failed to filter point cloud
          */
        virtual bool update(const T & input_pointcloud, T & output_pointcloud) = 0;

        /**
         * @brief Callback executed when a paramter change is detected
         * @param parameters list of changed parameters
         */
        virtual rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) = 0;

    private:
        template<typename PT>
        bool getParamFilterImpl(const std::string & name, const uint8_t type, PT default_value, PT & value_out)
        {
            std::string param_name = filters::FilterBase<T>::param_prefix_ + name;

            if (!filters::FilterBase<T>::params_interface_->has_parameter(param_name)) {
            // Declare parameter
            rclcpp::ParameterValue default_parameter_value(default_value);
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = name;
            desc.type = type;
            desc.read_only = false;

            if (name.empty()) {
                throw std::runtime_error("Parameter must have a name");
            }

            filters::FilterBase<T>::params_interface_->declare_parameter(param_name, default_parameter_value, desc);
            }

            rclcpp::Parameter local_value = (filters::FilterBase<T>::params_interface_->get_parameter(param_name));
            value_out = local_value.get_parameter_value().get<PT>();
            return true;
        }

    protected:
        /**
         * \brief Get a filter parameter as a string
         * \param name The name of the parameter
         * \param value The string to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, std::string & value)
        {
            return getParamFilterImpl(
            name, rcl_interfaces::msg::ParameterType::PARAMETER_STRING, std::string(), value);
        }

        /**
         * \brief Get a filter parameter as a boolean
         * \param name The name of the parameter
         * \param value The boolean to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, bool & value)
        {
            return getParamFilterImpl(name, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, false, value);
        }

        /**
         * \brief Get a filter parameter as a double
         * \param name The name of the parameter
         * \param value The double to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, double & value)
        {
            return getParamFilterImpl(name, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, 0.0, value);
        }

        /**
         * \brief Get a filter parameter as a int
         * \param name The name of the parameter
         * \param value The int to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, int & value)
        {
            return getParamFilterImpl(name, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, 0, value);
        }

        /**
         * \brief Get a filter parameter as an unsigned int
         * \param name The name of the parameter
         * \param value The int to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, unsigned int & value)
        {
            int signed_value;
            if (!getParamFilter(name, signed_value)) {
            return false;
            }
            if (signed_value < 0) {
            return false;
            }
            value = signed_value;
            return true;
        }

        /**
         * \brief Get a filter parameter as a size_t
         * \param name The name of the parameter
         * \param value The int to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, size_t & value)
        {
            int signed_value;
            if (!getParamFilter(name, signed_value)) {
            return false;
            }
            if (signed_value < 0) {
            return false;
            }
            value = signed_value;
            return true;
        }

        /**
         * \brief Get a filter parameter as a std::vector<double>
         * \param name The name of the parameter
         * \param value The std::vector<double> to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, std::vector<double> & value)
        {
            return getParamFilterImpl(
            name, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY, {}, value);
        }

        /**
         * \brief Get a filter parameter as a std::vector<string>
         * \param name The name of the parameter
         * \param value The std::vector<sgring> to set with the value
         * \return Whether or not the parameter of name/type was set
         */
        bool getParamFilter(const std::string & name, std::vector<std::string> & value)
        {
            return getParamFilterImpl(
            name, rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY, {}, value);
        }

        /**
         * @brief Print out params that are successfully set
         * @param name
         * @param value
         */
        void printParamUpdate(const std::string & name, const std::string & value) const
        {
            RCLCPP_INFO_STREAM(
                filters::FilterBase<T>::logging_interface_->get_logger(),
                name
                << " is updated with "
                << value
                << "."
            );
        }

        /// \brief The desired user filter field name.
        std::string filter_field_name_;

        /// \brief The minimum allowed filter value a point will be considered from.
        double filter_limit_min_;

        /// \brief The maximum allowed filter value a point will be considered from.
        double filter_limit_max_;

        /// \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false.
        bool filter_limit_negative_;

        /// \brief The input TF frame the data should be transformed into, if input.header.frame_id is different.
        std::string tf_input_frame_;

        /// \brief The original data input TF frame.
        std::string tf_input_orig_frame_;

        /// \brief The output TF frame the data should be transformed into, if input.header.frame_id is different.
        std::string tf_output_frame_;

        /// \brief Set whether the filtered points should be kept and set to NaN, or removed from the PointCloud, thus potentially breaking its organized structure.
        bool keep_organized_;

        /// \brief Dynamic parameters handler
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;


    };

} // namespace point_cloud_filters
