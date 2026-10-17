#include <gtest/gtest.h>
#include <limits>
#include "data_header/utility_function.hpp"
#include "data_header/generic_obstacle.hpp"

namespace
{
pcl::PointCloud<pcl::PointXYZI>::Ptr box(float length, float width, float height,
  float x = 5.0f, float y = 0.0f, float z = 0.0f)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (float dx = 0; dx <= length; dx += 0.15f) {
    for (float dy = 0; dy <= width; dy += 0.15f) {
      for (float dz = 0; dz <= height; dz += 0.15f) {
        pcl::PointXYZI p;
        p.x = x + dx; p.y = y + dy; p.z = z + dz; p.intensity = 1;
        cloud->push_back(p);
      }
    }
  }
  return cloud;
}

size_t detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr roi;
  return lidar::obstacleClusters(cloud, roi).size();
}
}  // namespace

TEST(GenericObstacle, AcceptsValidClustersAcrossSizes)
{
  EXPECT_EQ(1u, detect(box(0.6f, 0.6f, 0.8f)));  // drum
  EXPECT_EQ(1u, detect(box(0.3f, 0.45f, 1.65f)));  // human-sized surface
  EXPECT_EQ(1u, detect(box(4.5f, 1.8f, 1.5f)));  // vehicle-sized surface
  EXPECT_EQ(1u, detect(box(0.3f, 0.3f, 0.3f)));  // box, rejected by old gates
}

TEST(GenericObstacle, RejectsNoiseOutsideRoiAndGroundCrop)
{
  EXPECT_EQ(0u, detect(box(0, 0, 0)));
  EXPECT_EQ(0u, detect(box(0.15f, 0, 0)));  // fewer than three voxel points
  EXPECT_EQ(0u, detect(box(0.3f, 0.3f, 0.3f, 21)));
  EXPECT_EQ(0u, detect(box(0.3f, 0.3f, 0.3f, 5, 4)));
  EXPECT_EQ(0u, detect(box(0.3f, 0.3f, 0, 5, 0, -0.65f)));
  EXPECT_EQ(0u, detect(box(0.3f, 0.3f, 0.3f, 5, 0, 2)));
}

TEST(GenericObstacle, RejectsNonFiniteEvenWhenMarkedDense)
{
  auto cloud = box(0, 0, 0);
  cloud->points[0].x = std::numeric_limits<float>::quiet_NaN();
  cloud->is_dense = true;
  EXPECT_TRUE(lidar::finitePoints(cloud)->empty());
  EXPECT_EQ(0u, detect(cloud));
}
