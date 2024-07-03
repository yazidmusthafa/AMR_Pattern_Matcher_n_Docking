#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>
#include <matcher_action_interfaces/action/matcher.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <array>
#include <functional>
#include <memory>
#include <thread>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using PatternMatchPose = matcher_action_interfaces::action::Matcher;
using GoalHandlePatternMatchPose = rclcpp_action::ClientGoalHandle<PatternMatchPose>;

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode() : Node("robot_control_node")
    {
        // Subscriptions and Publishers
        initialize_charging = create_subscription<std_msgs::msg::Bool>(
            "/initialize_charging", 10, std::bind(&RobotControlNode::goToCharging, this, std::placeholders::_1));
        
        encoder_data = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&RobotControlNode::back_dock, this, std::placeholders::_1));
        
        velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        // goal_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        matcher_action_client = rclcpp_action::create_client<PatternMatchPose>(this, "pattern_match");

        local_costmap_parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/local_costmap/local_costmap");
        global_costmap_parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_costmap/global_costmap");

        send_goal_options.result_callback = std::bind(&RobotControlNode::result_callback, this, std::placeholders::_1);
        
        send_initiate_.result_callback = std::bind(&RobotControlNode::matcher_result_callback, this, std::placeholders::_1);

        // Create a publisher to publish Twist messages
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Action Servers:    /pattern_match: matcher_action_interfaces/action/Matcher

        
        
    }

    void publish_velocity()
    {
        // Loop to move the robot for 2 seconds
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 10)
        {   
            // Set the forward velocity
            vel_msg_.linear.x = 0.124;  // Move forward with 0.2 m/s
            vel_msg_.angular.z = 0.0; // No rotation

            // Publish the velocity message
            publisher_->publish(vel_msg_);
            // Sleep to maintain the loop rate
            RCLCPP_INFO(this->get_logger(), "started moving");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop the robot after 2 seconds
        vel_msg_.linear.x = 0.0;
        publisher_->publish(vel_msg_);

        RCLCPP_INFO(this->get_logger(), "Robot moved forward for 2 seconds");
    }

private:

    void back_dock(const sensor_msgs::msg::JointState::SharedPtr enc){

        double left_wheel = enc->position[0];
        double right_wheel = enc->position[1];

        // RCLCPP_INFO(this->get_logger(), "Left Wheel Encoder Data ###### %s", dock ? "true" : "false");

        if (backdock == true && initial_point == false){
            left_start = left_wheel;
            right_start = right_wheel;   
            initial_point = true;      
        }

        if (backdock){

            if ((left_wheel-left_start) <= 13.7195){
                vel_msg_.linear.x = 0.0; 
                vel_msg_.angular.z = -0.1;
                publisher_->publish(vel_msg_);
                // RCLCPP_INFO(this->get_logger(), "Left Wheel Encoder Data ###### %f", left_wheel);
            }
            else{
                vel_msg_.linear.x = 0.0; 
                vel_msg_.angular.z = 0.0;
                publisher_->publish(vel_msg_);

                final_dock = true;

                RCLCPP_INFO(this->get_logger(), "Left Wheel Encoder Data Diff ###### %f", (left_wheel-left_start));
            }

            if (final_dock){
            
                if((left_wheel-left_start) >= 11.2195){
                    vel_msg_.linear.x = -0.1; 
                    vel_msg_.angular.z = 0.0;
                    publisher_->publish(vel_msg_);
                }
                else{
                    vel_msg_.linear.x = 0.0; 
                    vel_msg_.angular.z = 0.0;
                    publisher_->publish(vel_msg_);

                    dock = false;     /////////
                    backdock = false;
                    initial_point = true;

                    RCLCPP_INFO(this->get_logger(), "Dock Completed with Left Wheel Encoder Data Diff ###### %f", (left_wheel-left_start));
                }
            }

        }
    
    }

    void goToCharging(const std_msgs::msg::Bool::SharedPtr x){  
        
        //ros2 topic pub --once /initialize_charging std_msgs/msg/Bool data:\ true\ 

        if (x){

            nav_action_client_->wait_for_action_server();

            geometry_msgs::msg::PoseStamped gl_pose;      

            gl_pose.header.frame_id = "map";
            gl_pose.pose.position.x = 1.0838078260421753;
            gl_pose.pose.position.y = 0.009256;
            gl_pose.pose.position.z = 0.0;
            gl_pose.pose.orientation.x = 0.0;
            gl_pose.pose.orientation.y = 0.0;
            gl_pose.pose.orientation.z = 0.001975811161024301;
            gl_pose.pose.orientation.w = 1.0;
            // goal_pose->publish(gl_pose);
            goal_msg.pose = gl_pose;
            
            nav_action_client_->async_send_goal(goal_msg, send_goal_options);

        }
    }
    
    void matcher_result_callback(const GoalHandlePatternMatchPose::WrappedResult &results){
        pattern_pose = results.result->result_pose;
        goal_msg.pose = pattern_pose;
        nav_action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(get_logger(), "******* Matcher result callback Run ******");
        RCLCPP_INFO(get_logger(), "Pattern matching succeeded. Result Pose: x=%f, y=%f, z=%f",
                        pattern_pose.pose.position.x, pattern_pose.pose.position.y, pattern_pose.pose.position.z);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded");
            if (dock == false){

                global_costmap_parameters_client->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", 0.1)});
                global_costmap_parameters_client->set_parameters({rclcpp::Parameter("inflation_layer.cost_scaling_factor", 5.0)});
                local_costmap_parameters_client->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", 0.05)});
                local_costmap_parameters_client->set_parameters({rclcpp::Parameter("inflation_layer.cost_scaling_factor", 5.0)});

                nav_action_client_->wait_for_action_server();

                matcher_action_client -> wait_for_action_server();

                initiate_msg.initialize = true;

                matcher_action_client->async_send_goal(initiate_msg, send_initiate_);

                dock = true;

            }
            else{

                // Add blind docking command
                RCLCPP_INFO(get_logger(), "**************Final Step*************");

                backdock = true;

            //    publish_velocity();

            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Goal aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal canceled");
            return;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            return;
        }
    }

    bool dock = false;
    bool backdock = false;
    bool final_dock = false;
    double left_start;
    double right_start;
    bool initial_point = false;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::PoseStamped pattern_pose;
    geometry_msgs::msg::Twist vel_msg_;
    rclcpp::Time start_time_;
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
    rclcpp_action::Client<PatternMatchPose>::SendGoalOptions send_initiate_;
    std::shared_ptr<rclcpp::AsyncParametersClient> local_costmap_parameters_client;
    std::shared_ptr<rclcpp::AsyncParametersClient> global_costmap_parameters_client;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp_action::Client<PatternMatchPose>::SharedPtr matcher_action_client;
    PatternMatchPose::Goal initiate_msg;
    NavigateToPose::Goal goal_msg;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr initialize_charging;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_data;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose;
};


//    ******************************** CLASS 2: PatternMatcherNode ******************************** 


class PatternMatcherNode : public rclcpp::Node
{
    public:
    using Matcher = matcher_action_interfaces::action::Matcher;
    using GoalHandleMatcher = rclcpp_action::ServerGoalHandle<Matcher>;

    PatternMatcherNode() : Node("pattern_matcher_node")
    {
                // Load pattern PCD file
        std::string pattern_path = ament_index_cpp::get_package_share_directory("pattern_matcher") + "/pcd/pattern_1.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pattern_path, *pattern_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read pattern file");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded pattern PCD file with %zu points", pattern_cloud_->points.size());


        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PatternMatcherNode::scanCallback, this, std::placeholders::_1));
        
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan/pointcloud", 10);
        pattern_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan/pattern_matched", 10);
        pattern_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pattern_pose", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        initial_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        using namespace std::placeholders;

        matcher_action_server_ = rclcpp_action::create_server<Matcher>(
            this,
            "pattern_match",
            std::bind(&PatternMatcherNode::handle_goal, this, _1, _2),
            std::bind(&PatternMatcherNode::handle_cancel, this, _1),
            std::bind(&PatternMatcherNode::handle_accepted, this, _1));

    }

    private:

    rclcpp_action::Server<Matcher>::SharedPtr matcher_action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Matcher::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request ");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMatcher> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMatcher> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&PatternMatcherNode::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMatcher> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        rclcpp::Rate loop_rate(4);                                // Important for long run processes

        auto result = std::make_shared<Matcher::Result>();

        pattern_pose_in_map_mean.pose.position.x = 0.0;
        pattern_pose_in_map_mean.pose.position.y = 0.0;
        pattern_pose_in_map_mean.pose.position.z = 0.0;
        pattern_pose_in_map_mean.pose.orientation.x = 0.0;
        pattern_pose_in_map_mean.pose.orientation.y = 0.0;
        pattern_pose_in_map_mean.pose.orientation.z = 0.0;
        pattern_pose_in_map_mean.pose.orientation.w = 0.0;

        for (int i = 0; i < 100; ++i)
        {
            if (goal_handle->is_canceling())
            {
                // result->result_pose = /* fill with appropriate data */;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // Convert LaserScan to PointCloud
            float angle = scan_msg->angle_min;
            for (const auto &range : scan_msg->ranges)
            {
                if (range >= scan_msg->range_min && range <= 1.55)
                {
                    pcl::PointXYZ point;
                    point.x = range * std::cos(angle);
                    point.y = range * std::sin(angle);
                    point.z = 0.0;
                    cloud->points.push_back(point);
                }
                angle += scan_msg->angle_increment;
            }

            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
            // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

            // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
            // mls.setComputeNormals(false);
            // mls.setInputCloud(cloud);
            // mls.setPolynomialOrder(2);
            // mls.setSearchMethod(tree);
            // mls.setSearchRadius(0.05);

            // mls.process(*cloud_smoothed);

            // Perform Euclidean Clustering
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.06); 
            ec.setMinClusterSize(14);
            ec.setMaxClusterSize(120);
            ec.setInputCloud(cloud);       // cloud_smoothed
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

                // RCLCPP_INFO(get_logger(), "######### Size of ####### %li", cloud_cluster->points.size());



                // Perform ICP for pattern matching
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                icp.setInputSource(pattern_cloud_);
                icp.setInputTarget(cloud_cluster);
                icp.setMaximumIterations(3000); // Increase iterations for better convergence

                // icp.setRANSACIterations(1000);
                // icp.setTransformationRotationEpsilon(1.0E-8);

                pcl::PointCloud<pcl::PointXYZ> Final;
                icp.align(Final);

                if (icp.hasConverged() && icp.getFitnessScore() < 0.00015)
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

                    // Broadcast the transform
                    geometry_msgs::msg::PoseStamped pattern_pose;
                    
                    pattern_pose.header.stamp = transform_stamped.header.stamp = this->get_clock()->now();
                    pattern_pose.header.frame_id = transform_stamped.header.frame_id = "base_link"; // Adjust the frame according to your setup
                    transform_stamped.child_frame_id = "pattern_frame";
                    pattern_pose.pose.position.x = transform_stamped.transform.translation.x = best_transformation(0, 3);
                    pattern_pose.pose.position.y = transform_stamped.transform.translation.y = best_transformation(1, 3);
                    pattern_pose.pose.position.z = transform_stamped.transform.translation.z = best_transformation(2, 3);

                    Eigen::Quaternionf q(best_transformation.block<3, 3>(0, 0));
                    pattern_pose.pose.orientation.x = transform_stamped.transform.rotation.x = q.x();
                    pattern_pose.pose.orientation.y = transform_stamped.transform.rotation.y = q.y();
                    pattern_pose.pose.orientation.z = transform_stamped.transform.rotation.z = q.z();
                    pattern_pose.pose.orientation.w = transform_stamped.transform.rotation.w = q.w();

                    pattern_pose_in_map = tf_buffer_->transform(pattern_pose, "map", tf2::Duration(std::chrono::seconds(2)));

                    /////////////////////////

                    transform_stamped_2.header.stamp = pattern_pose_in_map.header.stamp;
                    transform_stamped_2.header.frame_id = pattern_pose_in_map.header.frame_id;
                    transform_stamped_2.transform.translation.x = pattern_pose_in_map.pose.position.x;
                    transform_stamped_2.transform.translation.y = pattern_pose_in_map.pose.position.y;
                    transform_stamped_2.transform.translation.z = pattern_pose_in_map.pose.position.z;
                    transform_stamped_2.transform.rotation.x = pattern_pose_in_map.pose.orientation.x;
                    transform_stamped_2.transform.rotation.y = pattern_pose_in_map.pose.orientation.y;
                    transform_stamped_2.transform.rotation.z = pattern_pose_in_map.pose.orientation.z;
                    transform_stamped_2.transform.rotation.w = pattern_pose_in_map.pose.orientation.w;
                    transform_stamped_2.child_frame_id = "pattern_frame";

                    ///////////////////////////

                    tf_broadcaster_->sendTransform(transform_stamped_2);

                    pattern_pose_in_map_mean.pose.position.x += pattern_pose_in_map.pose.position.x;
                    pattern_pose_in_map_mean.pose.position.y += pattern_pose_in_map.pose.position.y;
                    pattern_pose_in_map_mean.pose.position.z += pattern_pose_in_map.pose.position.z;
                    pattern_pose_in_map_mean.pose.orientation.x += pattern_pose_in_map.pose.orientation.x;
                    pattern_pose_in_map_mean.pose.orientation.y += pattern_pose_in_map.pose.orientation.y;
                    pattern_pose_in_map_mean.pose.orientation.z += pattern_pose_in_map.pose.orientation.z;
                    pattern_pose_in_map_mean.pose.orientation.w += pattern_pose_in_map.pose.orientation.w;

                    // ros2 topic echo /tf | grep -B 4 -A 12 "child_frame_id: pattern_frame"

                }
            }

            loop_rate.sleep();

        }

        pattern_pose_in_map_mean.pose.position.x /= 10.0;
        pattern_pose_in_map_mean.pose.position.y /= 10.0;
        pattern_pose_in_map_mean.pose.position.z /= 10.0;
        pattern_pose_in_map_mean.pose.orientation.x /= 10.0;
        pattern_pose_in_map_mean.pose.orientation.y /= 10.0;
        pattern_pose_in_map_mean.pose.orientation.z /= 10.0;
        pattern_pose_in_map_mean.pose.orientation.w /= 10.0;

         // Normalize the quaternion
        double norm = std::sqrt(
        pattern_pose_in_map_mean.pose.orientation.x * pattern_pose_in_map_mean.pose.orientation.x +
        pattern_pose_in_map_mean.pose.orientation.y * pattern_pose_in_map_mean.pose.orientation.y +
        pattern_pose_in_map_mean.pose.orientation.z * pattern_pose_in_map_mean.pose.orientation.z +
        pattern_pose_in_map_mean.pose.orientation.w * pattern_pose_in_map_mean.pose.orientation.w);

        pattern_pose_in_map_mean.pose.orientation.x /= norm;
        pattern_pose_in_map_mean.pose.orientation.y /= norm;
        pattern_pose_in_map_mean.pose.orientation.z /= norm;
        pattern_pose_in_map_mean.pose.orientation.w /= norm;


        transform_stamped_2.header.stamp = pattern_pose_in_map_mean.header.stamp;
        transform_stamped_2.header.frame_id = pattern_pose_in_map_mean.header.frame_id;
        transform_stamped_2.transform.translation.x = pattern_pose_in_map_mean.pose.position.x;
        transform_stamped_2.transform.translation.y = pattern_pose_in_map_mean.pose.position.y;
        transform_stamped_2.transform.translation.z = pattern_pose_in_map_mean.pose.position.z;
        transform_stamped_2.transform.rotation.x = pattern_pose_in_map_mean.pose.orientation.x;
        transform_stamped_2.transform.rotation.y = pattern_pose_in_map_mean.pose.orientation.y;
        transform_stamped_2.transform.rotation.z = pattern_pose_in_map_mean.pose.orientation.z;
        transform_stamped_2.transform.rotation.w = pattern_pose_in_map_mean.pose.orientation.w;
        transform_stamped_2.child_frame_id = "pattern_frame";

        tf_broadcaster_->sendTransform(transform_stamped_2);

        result->result_pose = pattern_pose_in_map;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Pattern matched successfully");
    
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msgs)
    {

        if (initialize == true){

        geometry_msgs::msg::PoseWithCovarianceStamped init_pose;       // Try top make it automatically taking the transform of the robot base link
        init_pose.header.frame_id = "map";
        init_pose.pose.pose.position.x = 0.0;
        init_pose.pose.pose.position.y = 0.0;
        init_pose.pose.pose.position.z = 0.0;
        init_pose.pose.pose.orientation.x = 0.0;
        init_pose.pose.pose.orientation.y = 0.0;
        init_pose.pose.pose.orientation.z = 0.0;
        init_pose.pose.pose.orientation.w = 1.0;
        initial_pose->publish(init_pose);

        initialize = false;
        }

        scan_msg = scan_msgs;

    }

    bool initialize = true;
    sensor_msgs::msg::LaserScan::SharedPtr scan_msg;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pattern_pose_pub;
    geometry_msgs::msg::PoseStamped pattern_pose_in_map;
    geometry_msgs::msg::PoseStamped pattern_pose_in_map_mean;
    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::TransformStamped transform_stamped_2;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pattern_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pattern_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto scan_to_pointcloud_node = std::make_shared<RobotControlNode>();
    auto pattern_matcher_node = std::make_shared<PatternMatcherNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(scan_to_pointcloud_node);
    executor.add_node(pattern_matcher_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
