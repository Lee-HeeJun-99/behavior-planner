// Read serialized bag messages without creating ROS nodes or changing the source bag.
#include <sqlite3.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "data_header/generic_obstacle.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {std::cerr << "Usage: bag_obstacle_audit INPUT.db3 OUTPUT_DIRECTORY\n"; return 2;}
  sqlite3 * db = nullptr;
  if (sqlite3_open_v2(argv[1], &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {return 2;}
  sqlite3_stmt * query = nullptr;
  const char * sql = "SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
    "WHERE t.name='/velodyne_points' ORDER BY m.timestamp,m.id";
  if (sqlite3_prepare_v2(db, sql, -1, &query, nullptr) != SQLITE_OK) {return 2;}
  const std::string out = argv[2];
  std::ofstream frames(out + "/frames.csv"), clusters(out + "/clusters.csv");
  frames << "frame,timestamp_ns,input_points,finite_points,roi_points,clusters,old_gate_accepted,newly_accepted,processing_ms\n";
  clusters << "frame,cluster,points,x,y,z,length,width,height,min_z,max_z,old_gate_accepted\n";
  clusters << std::setprecision(10);
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  size_t frame = 0;
  try {
    while (sqlite3_step(query) == SQLITE_ROW) {
      const auto timestamp = sqlite3_column_int64(query, 0);
      const auto bytes = sqlite3_column_bytes(query, 1);
      rclcpp::SerializedMessage serialized(bytes);
      auto & data = serialized.get_rcl_serialized_message();
      std::memcpy(data.buffer, sqlite3_column_blob(query, 1), bytes);
      data.buffer_length = bytes;
      sensor_msgs::msg::PointCloud2 message;
      serializer.deserialize_message(&serialized, &message);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(message, *cloud);
      auto start = std::chrono::steady_clock::now();
      pcl::PointCloud<pcl::PointXYZI>::Ptr roi;
      const auto indices = lidar::obstacleClusters(cloud, roi);
      auto milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      std::vector<int> labels(roi->size(), -1);
      std::vector<int> old_gates(indices.size(), 0);
      size_t old_accepted = 0;
      for (size_t i = 0; i < indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZI> cluster;
        for (int index : indices[i].indices) {
          cluster.push_back(roi->points[index]); labels[index] = i;
        }
        Eigen::Vector4f minimum, maximum, center;
        pcl::getMinMax3D(cluster, minimum, maximum);
        pcl::compute3DCentroid(cluster, center);
        const double width = maximum.y() - minimum.y();
        const double length = maximum.x() - minimum.x();
        const bool small = width > 0.5 && width < 1.5 && maximum.z() < 0.3;
        const bool car = width >= 0.3 && width <= 5.5 && length <= 5.5 &&
          maximum.z() <= 1.8 && (width >= 0.8 || length >= 0.8);
        old_gates[i] = small || car;
        old_accepted += old_gates[i];
        clusters << frame << ',' << i << ',' << cluster.size() << ',' << center.x() << ','
          << center.y() << ',' << center.z() << ',' << length << ',' << width << ','
          << maximum.z() - minimum.z() << ',' << minimum.z() << ',' << maximum.z() << ','
          << old_gates[i] << '\n';
      }
      frames << frame << ',' << timestamp << ',' << cloud->size() << ','
        << lidar::finitePoints(cloud)->size() << ',' << roi->size() << ',' << indices.size()
        << ',' << old_accepted << ',' << indices.size() - old_accepted << ',' << milliseconds << '\n';
      if (frame == 0 || frame == 250 || frame == 500 || frame == 750 || frame == 1007) {
        std::ofstream sample(out + "/roi_" + std::to_string(frame) + ".csv");
        sample << "x,y,z,cluster,old_gate_accepted\n";
        for (size_t j = 0; j < roi->size(); ++j) {
          const auto & p = roi->points[j];
          sample << p.x << ',' << p.y << ',' << p.z << ',' << labels[j] << ','
            << (labels[j] >= 0 ? old_gates[labels[j]] : -1) << '\n';
        }
      }
      if (++frame % 100 == 0) {std::cerr << "Processed " << frame << " frames\n";}
    }
    if (sqlite3_errcode(db) != SQLITE_DONE && sqlite3_errcode(db) != SQLITE_OK) {
      throw std::runtime_error(sqlite3_errmsg(db));
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; return 1;
  }
  sqlite3_finalize(query);
  sqlite3_close(db);
  std::cout << "Processed all " << frame << " raw LiDAR frames\n";
  return frame ? 0 : 1;
}
