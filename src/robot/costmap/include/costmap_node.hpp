#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include <vector>
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // callback function for laser scan
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
 
  private:
    robot::CostmapCore costmap_;
    
    // helper functions
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();
    
    // ROS constructs
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    
    // costmap parameters
    double resolution_;
    int width_;
    int height_;
    double inflation_radius_;
    // costmap data
    std::vector<int8_t> costmap_data_;
};
 
#endif