#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"
#include <cmath>

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();
    
private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    
    void integrateCostmap();
    void publishCurrentMap();
    double distanceMoved();
    
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid current_costmap_;
    
    double last_x;
    double last_y;
    double curr_x;
    double curr_y;
    double move_threshold;
    bool got_costmap;
    bool need_update;
    

    
    int map_size;
    double map_res;
};

#endif