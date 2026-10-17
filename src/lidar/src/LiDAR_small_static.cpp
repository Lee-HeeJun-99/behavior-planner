#include "mission_header/LiDAR_small_static.hpp"
#include "data_header/generic_obstacle.hpp"

LidarObstacleDetector::LidarObstacleDetector()
    : Mission("lidar_obstacle_detector")
{
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/obstacles", lidar::best_effort_qos());
  object_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/object_cen", lidar::best_effort_qos());
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/obstacle_roi", lidar::best_effort_qos());
  marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/obstacles", lidar::best_effort_qos());

  RCLCPP_INFO(this->get_logger(), "Generic obstacle detector is ready");
}

void LidarObstacleDetector::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  if (check_pointcloud == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Generic obstacle detector received pointcloud");
    check_pointcloud = 1;
  }

  // 1. 변환
  auto cloud = util_func_.PointCloudROS(input);

  const auto input_count = cloud->size();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2;
  auto cluster_indices = lidar::obstacleClusters(cloud, cloud_filtered2);

  // 5. 클러스터 추출 및 색 입히기
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
  util_func_.ExtractClusters(TotalCloud, cloud_filtered2, cluster_indices, clusters);

  std_msgs::msg::Float64MultiArray obj;

  for (size_t i = 0; i < clusters.size(); i++)
  {
    geometry_msgs::msg::Point center_point, min_point, max_point;
    util_func_.GetSenCloudPoint(clusters, center_point, i);
    util_func_.GetMinMaxCloudPoint(clusters, min_point, max_point, i);

    float cen_x = center_point.x;
    float cen_y = center_point.y;

    // Euclidean extraction already enforces the minimum post-voxel point count.
    // No width/length/height category gates: all finite ROI clusters are obstacles.
    obj.data.push_back(cen_x);
    obj.data.push_back(cen_y);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Obstacle detection: input=%zu roi=%zu clusters=%zu accepted generic obstacles=%zu",
    input_count, cloud_filtered2->size(), clusters.size(), obj.data.size() / 2);
  obj.data.push_back(-1000);
  object_->publish(obj);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = input->header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  auto centers = util_func_.showRoi_POINT("obstacle_centers", 0);
  centers.header = input->header;
  for (size_t i = 0; i + 1 < obj.data.size(); i += 2) {
    geometry_msgs::msg::Point point;
    point.x = obj.data[i];
    point.y = obj.data[i + 1];
    centers.points.push_back(point);
  }
  markers.markers.push_back(centers);
  marker_array_->publish(markers);

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header = input->header;
  pub_->publish(output);
}
