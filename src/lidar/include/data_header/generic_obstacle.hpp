#ifndef LIDAR_GENERIC_OBSTACLE_HPP_
#define LIDAR_GENERIC_OBSTACLE_HPP_

#include <cmath>
#include "data_header/utility_function.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar
{
constexpr float kObstacleForwardRange = 5.0f;
constexpr float kObstacleRoadHalfWidth = 3.0f;
// Explicitly reject non-finite coordinates even when the driver marks is_dense=true.
inline pcl::PointCloud<pcl::PointXYZI>::Ptr finitePoints(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto & point : input->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      output->push_back(point);
    }
  }
  output->is_dense = true;
  return output;
}
// Shared production/test pipeline; thresholds describe cluster reliability, not classes.
inline std::vector<pcl::PointIndices> obstacleClusters(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  pcl::PointCloud<pcl::PointXYZI>::Ptr & roi)
{
  Utility_Function utility;
  auto finite = finitePoints(input);
  auto voxel = utility.DownSamplingVoxel(finite, 0.1f, 0.1f, 0.1f);
  // TODO: verify ROI covers actual road width and candidate area on vehicle.
  // Z crop is the existing ground suppression; no ground-plane segmentation.
  roi = utility.CropFilter(voxel, Eigen::Vector4f(0, -kObstacleRoadHalfWidth, -0.35, 0),
    Eigen::Vector4f(kObstacleForwardRange, kObstacleRoadHalfWidth, 1.8, 0));
  // max_ok=false intentionally preserves the existing absence of a size cap.
  return utility.ClusterEuclidean(roi, false, 0.4, 3, 20000);
}
}  // namespace lidar
#endif  // LIDAR_GENERIC_OBSTACLE_HPP_
