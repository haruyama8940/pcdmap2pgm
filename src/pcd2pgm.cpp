#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <string>

using namespace std::chrono_literals;

std::string file_directory;
std::string file_name;
std::string pcd_file;

std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::msg::OccupancyGrid::SharedPtr map_topic_msg;

double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;

double grid_x = 0.1;
double grid_y = 0.1;
double grid_z = 0.1;

double map_resolution = 0.05;

double thre_radius = 0.1;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void PassThroughFilter(const double &thre_low, const double &thre_high, const bool &flag_in);

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid::SharedPtr msg,rclcpp::Node::SharedPtr node);

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("convert_pcd");
  
    rclcpp::Clock ros_clock;
  
    rclcpp::Rate loop_rate(1.0);
  
    // rclcpp::Parameter parameter;
    /*
    node->get_parameter_or("file_directory", file_directory, std::string("/home/ubuntu/"));
    RCLCPP_INFO(node->get_logger(), "*** file_directory = %s ***", file_directory.c_str());
  
    node->get_parameter_or("file_name", file_name, std::string("good2"));
    RCLCPP_INFO(node->get_logger(), "*** file_name = %s ***", file_name.c_str());
    
    pcd_file = file_directory + file_name + pcd_format;
    RCLCPP_INFO(node->get_logger(), "*** pcd_file = %s ***", pcd_file.c_str());
    */
    node->declare_parameter("pcd_file","/kani.pcd");
    node->get_parameter("pcd_file",pcd_file);
    RCLCPP_INFO(node->get_logger(), "*** pcd_file = %s ***", pcd_file.c_str());
    node->get_parameter_or("thre_z_min", thre_z_min, 0.5);
    node->get_parameter_or("thre_z_max", thre_z_max, 2.0);
    node->get_parameter_or("flag_pass_through", flag_pass_through, 0);
    node->get_parameter_or("grid_x", grid_x, 0.1);
    node->get_parameter_or("grid_y", grid_y, 0.1);
    node->get_parameter_or("grid_z", grid_z, 0.1);
    node->get_parameter_or("thre_radius", thre_radius, 0.5);
    node->get_parameter_or("map_resolution", map_resolution, 0.05);
    node->get_parameter_or("map_topic_name", map_topic_name, std::string("map"));
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_topic_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_topic_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    map_topic_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name, 1);
    RCLCPP_INFO(node->get_logger(), "debug");
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *pcd_cloud) == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
        return -1;
    }
  
    RCLCPP_INFO(node->get_logger(), "Initial point cloud data points: %lu", pcd_cloud->points.size());
  
    PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through));
  
    SetMapTopicMsg(cloud_after_PassThrough, map_topic_msg,node);
  
    while (rclcpp::ok())
    {
        map_topic_pub->publish(*map_topic_msg);
  
        rclcpp::spin_some(node);
  
        loop_rate.sleep();
    }
  
    rclcpp::shutdown();
  
    return 0;
}

void PassThroughFilter(const double &thre_low, const double &thre_high, const bool &flag_in)
{
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(thre_low, thre_high);
    passthrough.setFilterLimitsNegative(flag_in);
    passthrough.filter(*cloud_after_PassThrough);
    RCLCPP_INFO(rclcpp::get_logger("PassThroughFilter"), "Point cloud data points after PassThrough: %lu", cloud_after_PassThrough->points.size());
}

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid::SharedPtr msg, rclcpp::Node::SharedPtr node)
{
    // msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
   
    rclcpp::Time current_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    // msg->header.stamp = ros_clock.now();
    msg->header.stamp = current_stamp;
    msg->header.frame_id = "map";
    RCLCPP_INFO(node->get_logger(), "degug setmap");
    // msg->info.map_load_time = ros_clock.now();
    msg->info.map_load_time = current_stamp;
    msg->info.resolution = map_resolution;
   
    double x_min, x_max, y_min, y_max;
  
    if (cloud->points.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("SetMapTopicMsg"), "PCD is empty!");
        return;
    }
    
    for (int i = 0; i < cloud->points.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = cloud->points[i].x;
            y_min = y_max = cloud->points[i].y;
        }
  
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
  
        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
  
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }
  
    msg->info.origin.position.x = x_min;
    msg->info.origin.position.y = y_min;
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.x = 0.0;
    msg->info.origin.orientation.y = 0.0;
    msg->info.origin.orientation.z = 0.0;
    msg->info.origin.orientation.w = 1.0;
  
    msg->info.width = int((x_max - x_min) / map_resolution);
    msg->info.height = int((y_max - y_min) / map_resolution);
  
    msg->data.resize(msg->info.width * msg->info.height);
    msg->data.assign(msg->info.width * msg->info.height, 0);
  
    RCLCPP_INFO(rclcpp::get_logger("SetMapTopicMsg"), "Data size = %lu", msg->data.size());
  
    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        int i = int((cloud->points[iter].x - x_min) / map_resolution);
        if (i < 0 || i >= msg->info.width) continue;
  
        int j = int((cloud->points[iter].y - y_min) / map_resolution);
        if (j < 0 || j >= msg->info.height - 1) continue;
  
        msg->data[i + j * msg->info.width] = 100;
    }
    RCLCPP_WARN(rclcpp::get_logger("SetMapTopicMsg"), "End setmaptopic");
}
