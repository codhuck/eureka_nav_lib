#include "eureka_nav_lib/eureka_nav_lib.hpp"
#include <rclcpp/rclcpp.hpp>

namespace eureka

{
    Localization::Localization() 
    : sec(0), nanosec(0),
      position({0.0, 0.0, 0.0}),
      orientation({0.0, 0.0, 0.0, 1.0}),
      position_v({0.0, 0.0, 0.0}),
      orientation_v({0.0, 0.0, 0.0}) {}

    Calculate_localization::Calculate_localization(rclcpp::Node::SharedPtr node): node(node) 
    {
    subscription = node->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&Calculate_localization::odometry_callback, this, std::placeholders::_1));
    }

    void Calculate_localization::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
    Localization loc;
    loc.sec = msg->header.stamp.sec;
    loc.nanosec = msg->header.stamp.nanosec;
    loc.position[0] = msg->pose.pose.position.x;
    loc.position[1] = msg->pose.pose.position.y;
    loc.position[2] = msg->pose.pose.position.z;
    loc.orientation[0] = msg->pose.pose.orientation.x;
    loc.orientation[1] = msg->pose.pose.orientation.y;
    loc.orientation[2] = msg->pose.pose.orientation.z;
    loc.orientation[3] = msg->pose.pose.orientation.w;
    loc.position_v[0] = msg->twist.twist.linear.x;
    loc.position_v[1] = msg->twist.twist.linear.y;
    loc.position_v[2] = msg->twist.twist.linear.z;
    loc.orientation_v[0] = msg->twist.twist.angular.x;
    loc.orientation_v[1] = msg->twist.twist.angular.y;
    loc.orientation_v[2] = msg->twist.twist.angular.z;
    localization_data.push_back(loc);
    }

    std::vector<Localization> Calculate_localization::get_location() 
    {
    return localization_data;
    }

    Mapping::Mapping(): sec(0), nanosec(0), position({0.0, 0.0, 0.0}), orientation({0.0, 0.0, 0.0, 1.0}), resolution(0.05f), width(0), height(0) {}

    Construction_map::Construction_map(rclcpp::Node::SharedPtr node)
    : node(node) 
    {
    subscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap", 10, std::bind(&Construction_map::map_callback, this, std::placeholders::_1));
    }

    void Construction_map::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
    {
    Mapping map;
    map.sec = msg->info.map_load_time.sec;
    map.nanosec = msg->info.map_load_time.nanosec;
    map.position[0] = msg->info.origin.position.x;
    map.position[1] = msg->info.origin.position.y;
    map.position[2] = msg->info.origin.position.z;
    map.orientation[0] = msg->info.origin.orientation.x;
    map.orientation[1] = msg->info.origin.orientation.y;
    map.orientation[2] = msg->info.origin.orientation.z;
    map.orientation[3] = msg->info.origin.orientation.w;
    map.resolution = msg->info.resolution;
    map.width = msg->info.width;
    map.height = msg->info.height;
    map.data.assign(msg->data.begin(), msg->data.end());
    map_data.push_back(map);
    }

    std::vector<Mapping> Construction_map::get_map() 
    {
    return map_data;
    }

    Navigation::Navigation(rclcpp::Node::SharedPtr node)
    : node(node) 
    {
    
    goal_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    init_publisher = node->create_publisher<std_msgs::msg::String>("/init", 10);
    }

    void Navigation::move_to(double x, double y, double z) 
    {
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    
    goal_msg.header.stamp = node->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = sin(z / 2.0);
    goal_msg.pose.orientation.w = cos(z / 2.0);
    goal_publisher->publish(goal_msg);
    }

    void Navigation::setup() 
    {
    auto init_msg = std_msgs::msg::String();
    init_msg.data = "start";
    init_publisher->publish(init_msg);
    }

}