#include "data_header/mission.hpp"
#include "data_header/utility_function.hpp"

class LiDAR_small_static : public Mission
{
public:
    LiDAR_small_static();

private:
    // func
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override;

    // pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr object_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_;

    // var
    int check_pointcloud = 0;

    Utility_Function util_func_;
};
