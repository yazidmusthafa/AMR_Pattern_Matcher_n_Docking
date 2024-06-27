#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "laser_geometry/laser_geometry.hpp"

class PatternRecorder : public rclcpp::Node
{
public:
  PatternRecorder() : Node("pattern_recorder")
  {
    this->declare_parameter("mean_queue_size", 100);      //100
    this->declare_parameter("location_filter_x", 1.0);
    this->declare_parameter("location_filter_y", 0.5);
    this->declare_parameter("voxel_leaf_size", 0.0075f);
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("output_file", "/home/yazid_musthafa/turtlebot3_ws/src/pattern_matcher/pcd/pattern_1.pcd");

    mean_queue_size_ = this->get_parameter("mean_queue_size").as_int();
    location_filter_x_ = this->get_parameter("location_filter_x").as_double();
    location_filter_y_ = this->get_parameter("location_filter_y").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    output_file_ = this->get_parameter("output_file").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, std::bind(&PatternRecorder::scan_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Collecting Scans");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(167), std::bind(&PatternRecorder::timer_callback, this));
  }

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_msgs_.push_back(*msg);
  }

  void timer_callback()
  {
    if (scan_msgs_.size() >= mean_queue_size_)
    {
      RCLCPP_INFO(this->get_logger(), "Collected Scans");
      sensor_msgs::msg::LaserScan final = calculateMeanScan(scan_msgs_);
      pcl::PointCloud<pcl::PointXYZ> cloud = scanToPCL(final);
      locationFilter(cloud, location_filter_x_, location_filter_y_);
      voxelFilter(cloud, voxel_leaf_size_);
      pcl::io::savePCDFileASCII(output_file_, cloud);
      RCLCPP_INFO(this->get_logger(), "Cloud written to %s", output_file_.c_str());
      rclcpp::shutdown();
    }
  }

  sensor_msgs::msg::LaserScan calculateMeanScan(std::vector<sensor_msgs::msg::LaserScan> &scans)
  {
    sensor_msgs::msg::LaserScan mean_scan;

    //  RCLCPP_INFO(this->get_logger(), "Collected Scans %li", scans[0].ranges.size());
    
    int scan_count = scans[0].ranges.size();

    for(size_t i = 0; i < scans.size(); i++)
    {
      if(i == 0)
      {
        mean_scan = scans[i];
        continue;
      }

      for(int j = 0; j < scan_count; j++)
      {
        mean_scan.ranges[j] += scans[i].ranges[j];
      }
    }

    for(auto &range : mean_scan.ranges)
    {
      range /= scans.size();
    }

    return mean_scan;
  }

  pcl::PointCloud<pcl::PointXYZ> scanToPCL(sensor_msgs::msg::LaserScan &scan)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::msg::PointCloud2 pointCloud;
    laser_geometry::LaserProjection laser_projector;
    laser_projector.projectLaser(scan, pointCloud);
    pcl::fromROSMsg (pointCloud, cloud);
    return cloud;
  }

  void locationFilter(pcl::PointCloud<pcl::PointXYZ> &cloud, double filter_x, double filter_y)
  {
    try
    {
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      int count = 0;
      for(size_t i = 0; i < cloud.size(); i++)
      {
        if(abs(cloud.points[i].x) > filter_x || abs(cloud.points[i].y) > filter_y)
        {
          inliers->indices.push_back(i);
          count++;
        }
      }
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud.makeShared());
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(cloud);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
  }

  void voxelFilter(pcl::PointCloud<pcl::PointXYZ> &cloud, double leaf_size)
  {
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(cloud, *cloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*cloud2);
    pcl::fromPCLPointCloud2(*cloud2, cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<sensor_msgs::msg::LaserScan> scan_msgs_;
  int mean_queue_size_;
  double location_filter_x_;
  double location_filter_y_;
  double voxel_leaf_size_;
  std::string scan_topic_;
  std::string output_file_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatternRecorder>());
  rclcpp::shutdown();
  return 0;
}
