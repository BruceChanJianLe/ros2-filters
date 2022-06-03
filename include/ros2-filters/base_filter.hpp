#pragma once

// STL
#include <string>

// REMOVE
// BOOST
// #include "boost/thread/recursive_mutex.hpp"

namespace point_cloud_filters
{
    class BaseFilter
    {
    public:
        BaseFilter();
        virtual ~BaseFilter();

    protected:
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

        // REMOVE
        /// \brief recursive mutex
        // boost::recursive_mutex m_;
    };

} // namespace point_cloud_filters
