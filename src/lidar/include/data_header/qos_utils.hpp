// qos_utils.hpp

#ifndef LIDAR_QOS_UTILS_HPP_
#define LIDAR_QOS_UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

namespace lidar
{

    // QoS settings for robust network
    inline rclcpp::QoS reliable_qos()
    {
        return rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()
            .durability_volatile();
            // .deadline(rclcpp::Duration(1, 0)) // 1 second
            // .lifespan(rclcpp::Duration(5, 0)) // 5 seconds
            // .liveliness(rclcpp::LivelinessPolicy::Automatic)
            // .liveliness_lease_duration(rclcpp::Duration(2, 0)); // 2 seconds
    }

    // QoS settings for non-robust network
    inline rclcpp::QoS best_effort_qos()
    {
        return rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();
        // .deadline(rclcpp::Duration(0.5, 0)) // 0.5 seconds
        // .lifespan(rclcpp::Duration(2, 0)) // 2 seconds
        // .liveliness(rclcpp::LivelinessPolicy::ManualByTopic)
        // .liveliness_lease_duration(rclcpp::Duration(2, 0)); // 5 seconds
    }

} // namespace lidar

#endif // LIDAR_QOS_UTILS_HPP_