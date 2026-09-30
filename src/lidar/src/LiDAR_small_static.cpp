#include "mission_header/LiDAR_small_static.hpp"

LiDAR_small_static::LiDAR_small_static()
    : Mission("LiDAR_small_static")
{
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/small_static", lidar::best_effort_qos());
  object_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/object_cen", lidar::best_effort_qos());
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/small_static_ROI", lidar::best_effort_qos());
  marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/small_static_object", lidar::best_effort_qos());

  RCLCPP_INFO(this->get_logger(), "LiDAR_small_static is ready");
}

void LiDAR_small_static::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  if (check_pointcloud == 0)
  {
    RCLCPP_INFO(this->get_logger(), "LiDAR_small_static node sub pointcloud");
    check_pointcloud = 1;
  }

  // 1. 변환
  auto cloud = util_func_.PointCloudROS(input);

  // 2. 다운샘플링
  auto cloud_filtered = util_func_.DownSamplingVoxel(cloud, 0.1f, 0.1f, 0.1f);

  // 3. Crop 필터
  auto cloud_filtered2 = util_func_.CropFilter(
      cloud_filtered,
      Eigen::Vector4f(-0, -2.5, -0.35, 0), // ROI 영역(LiDAR 높이(z축)가 지면으로부터 0.65m일 때)
      Eigen::Vector4f(15, 2, 1.0, 0));     // x, y, z, 1

  //////////////////////// < ROI를 나타내는 직육면체 마커 > //////////////////////////////////
  // visualization_msgs::msg::Marker marker;
  // marker.header.frame_id = "velodyne";
  // marker.ns = "my_roi";
  // marker.id = 0; // ID는 0으로 설정 (하나의 마커만 사용)
  // marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  // marker.action = visualization_msgs::msg::Marker::ADD;
  // marker.type = visualization_msgs::msg::Marker::CUBE;
  // marker.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
  // marker.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
  // marker.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
  // marker.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
  // marker.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
  // marker.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
  // marker.color.r = 0.0;
  // marker.color.g = 1.0;
  // marker.color.b = 0.0;
  // marker.color.a = 0.3;
  // marker_pub_->publish(marker);
  /////////////////////////////////////////////////////////////////////////////////////

  // 4. 클러스터링
  auto cluster_indices = util_func_.ClusterEuclidean(cloud_filtered2, false, 0.3, 3, 1000);

  // 5. 클러스터 추출 및 색 입히기
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
  util_func_.ExtractClusters(TotalCloud, cloud_filtered2, cluster_indices, clusters);

  std_msgs::msg::Float64MultiArray obj;
  std_msgs::msg::Float32MultiArray p;

  for (size_t i = 0; i < clusters.size(); i++)
  {
    geometry_msgs::msg::Point center_point, min_point, max_point;
    util_func_.GetSenCloudPoint(clusters, center_point, i);
    util_func_.GetMinMaxCloudPoint(clusters, min_point, max_point, i);

    float cen_x = center_point.x;
    float cen_y = center_point.y;

    // 좌우(y축) 폭이 0.5m 초과 && 1.5m 미만  &&  상하(z축) 높이가 0.95m 미만 (LiDAR 높이(z축)가 지면으로부터 0.65m일 때)
    if ((max_point.y - min_point.y) > 0.5 && (max_point.y - min_point.y) < 1.5 && max_point.z < 0.3)
    {
      obj.data.push_back(cen_x);
      obj.data.push_back(cen_y);
      p.data.push_back(cen_x);
      p.data.push_back(cen_y);
    }
  }

  obj.data.push_back(-1000);
  object_->publish(obj);

  p.data.push_back(-1000);

  visualization_msgs::msg::MarkerArray p_array;

  if (p.data.size() >= 4)
  {
    geometry_msgs::msg::Point obj1, obj2;
    visualization_msgs::msg::Marker marker1, marker2;

    marker1.ns = "points_and_lines";
    marker1.action = visualization_msgs::msg::Marker::ADD;
    marker1.type = visualization_msgs::msg::Marker::POINTS;
    marker1.id = 0;
    marker1.pose.orientation.w = 1.0;
    marker1.scale.x = 0.5;
    marker1.scale.y = 0.5;
    marker1.color.a = 1.0;
    marker1.color.r = 1.0f;
    obj1.x = p.data[0];
    obj1.y = p.data[1];
    obj1.z = 0.0;
    marker1.points.push_back(obj1);
    marker1.header.frame_id = "velodyne";
    p_array.markers.push_back(marker1);

    marker2.ns = "points_and_lines";
    marker2.action = visualization_msgs::msg::Marker::ADD;
    marker2.type = visualization_msgs::msg::Marker::POINTS;
    marker2.id = 1;
    marker2.pose.orientation.w = 1.0;
    marker2.scale.x = 0.5;
    marker2.scale.y = 0.5;
    marker2.color.a = 1.0;
    marker2.color.g = 1.0f;
    obj2.x = p.data[2];
    obj2.y = p.data[3];
    obj2.z = 0.0;
    marker2.points.push_back(obj2);
    marker2.header.frame_id = "velodyne";
    p_array.markers.push_back(marker2);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Not enough points for marker visualization (p.data.size() = %zu)", p.data.size());
  }

  marker_array_->publish(p_array);

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";
  pub_->publish(output);
}
