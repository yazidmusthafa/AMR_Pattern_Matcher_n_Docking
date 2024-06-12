#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <cmath>
#include <vector>
#include <array>

class ScanToPointCloudNode : public rclcpp::Node
{
public:
    ScanToPointCloudNode() : Node("pattern_matcher")
    {
        // Load pattern PCD file
        std::string pattern_path = ament_index_cpp::get_package_share_directory("pattern_matcher") + "/pcd/pattern.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pattern_path, *pattern_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read pattern file");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded pattern PCD file with %zu points", pattern_cloud_->points.size());

        // Subscriptions and Publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToPointCloudNode::scanCallback, this, std::placeholders::_1));
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan/pointcloud", 10);
        pattern_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan/pattern_matched", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Convert LaserScan to PointCloud
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        bool pattern_matched = false;
        Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();

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
            cloud_cluster->clear();
            for (const auto &idx : indices.indices)
            {
                pcl::PointXYZRGB point_rgb;
                pcl::PointXYZ point;
                point.x = point_rgb.x = cloud->points[idx].x;
                point.y = point_rgb.y = cloud->points[idx].y;
                point.z = point_rgb.z = cloud->points[idx].z;
                point_rgb.r = r;
                point_rgb.g = g;
                point_rgb.b = b;
                colored_cloud->points.push_back(point_rgb);
                cloud_cluster->points.push_back(point);
            }
            color_idx++;

            colored_cloud->width = colored_cloud->points.size();
            colored_cloud->height = 1;
            colored_cloud->is_dense = true;

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*colored_cloud, output);
            output.header = scan_msg->header;
            pc_pub_->publish(output);

            // Perform ICP for pattern matching
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(pattern_cloud_);
            icp.setInputTarget(cloud_cluster);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);

            if (icp.hasConverged() && icp.getFitnessScore() < 0.0001)
            {
                pattern_matched = true;
                best_transformation = icp.getFinalTransformation();
                // break; // Stop after finding the first match
            }
        

            if (pattern_matched)
            {
                // Transform pattern point cloud to match the detected location
                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pattern(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud(*pattern_cloud_, *transformed_pattern, best_transformation);

                // Publish the matched pattern point cloud
                sensor_msgs::msg::PointCloud2 pattern_output;
                pcl::toROSMsg(*transformed_pattern, pattern_output);
                pattern_output.header = scan_msg->header;
                pattern_pub_->publish(pattern_output);
            }

        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pattern_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pattern_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanToPointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
