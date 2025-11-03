#ifndef EUREKA_NAV_LIB_HPP
#define EUREKA_NAV_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <array>
#include <cstdint>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace eureka
{
struct Localization
{
    int32_t sec;
    int32_t nanosec;
    std::array<double, 3> position;
    std::array<double, 4> orientation;
    std::array<double, 3> position_v;
    std::array<double, 3> orientation_v;

    Localization();
};

class Calculate_localization
{
    private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
    std::vector<Localization> localization_data;
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    public:
    Calculate_localization(rclcpp::Node::SharedPtr node);
    std::vector<Localization> get_location();
};

struct Mapping
{
    int32_t sec;
    int32_t nanosec;
    std::array<double, 3> position;
    std::array<double, 4> orientation;
    float resolution;
    uint32_t width;
    uint32_t height;
    std::vector<int8_t> data;

    Mapping();
};

class Construction_map
{
    private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription;
    std::vector<Mapping> map_data;
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    public:
    Construction_map(rclcpp::Node::SharedPtr node);
    std::vector<Mapping> get_map();
};

class Navigation
{
    private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_publisher;
    public:
    Navigation(rclcpp::Node::SharedPtr node);
    void move_to(double x, double y, double z);
    void setup();
};

}
#endif