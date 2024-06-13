#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>
#include <vector>
#include <array>

class ScanToPointCloudNode : public rclcpp::Node
{
public:
    ScanToPointCloudNode() : Node("pattern_matcher")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToPointCloudNode::scanCallback, this, std::placeholders::_1));
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan/pointcloud", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        float angle = scan_msg->angle_min;
        for (const auto &range : scan_msg->ranges)
        {
            if (range >= scan_msg->range_min && range <= scan_msg->range_max)
            {
                pcl::PointXYZ point;
                point.x = range * std::cos(angle);
                point.y = range * std::sin(angle);
                point.z = 0.0;
                cloud->points.push_back(point);
            }
            angle += scan_msg->angle_increment;
        }

        // Perform Euclidean Clustering
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.06); // 2cm
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(500);
    // ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Create PointCloud2 message for publishing
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Define some colors for clusters
    std::vector<std::array<int, 3>> colors = {
        {255, 0, 0},    // Red
        {0, 255, 0},    // Green
        {0, 0, 255},    // Blue
        {255, 255, 0},  // Yellow
        {0, 255, 255},  // Cyan
        {255, 0, 255}   // Magenta
    };

    int color_idx = 0;
    for (const auto &indices : cluster_indices)
    {
        int r = colors[color_idx % colors.size()][0];
        int g = colors[color_idx % colors.size()][1];
        int b = colors[color_idx % colors.size()][2];
        for (const auto &idx : indices.indices)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[idx].x;
            point.y = cloud->points[idx].y;
            point.z = cloud->points[idx].z;
            point.r = r;
            point.g = g;
            point.b = b;
            colored_cloud->points.push_back(point);
        }
        color_idx++;
    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = scan_msg->header;
    pc_pub_->publish(output);
}

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanToPointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}