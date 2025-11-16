#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() 
    : Node("costmap"), 
      costmap_(robot::CostmapCore(this->get_logger())), resolution_(0.1), width_(200), height_(200), inflation_radius_(0.5) 
{
    // Initialize the subscriber to laser scan data
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 
        10, 
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
    );
    
    // Initialize the publisher for costmap
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 
        10
    );
    
    // Initialize costmap data
    costmap_data_.resize(width_ * height_, 0);
    
    RCLCPP_INFO(this->get_logger(), "Costmap Node initialized!");
    RCLCPP_INFO(this->get_logger(), "Grid: %dx%d cells, Resolution: %.2fm, Inflation: %.2fm", 
                width_, height_, resolution_, inflation_radius_);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

    initializeCostmap();
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        
        if (range >= scan->range_min && range <= scan->range_max) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            
            if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
                markObstacle(x_grid, y_grid);
            }
        }
    }
    
      // RCLCPP_DEBUG(this->get_logger(), "Detected %d obstacle points", obstacle_count);

    inflateObstacles();
    
    publishCostmap();
}

void CostmapNode::initializeCostmap() {
    std::fill(costmap_data_.begin(), costmap_data_.end(), 0);
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {

    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>((x / resolution_) + (width_ / 2));
    y_grid = static_cast<int>((y / resolution_) + (height_ / 2));
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    // convert 2d grid coordinates to 1d index
    int index = y_grid * width_ + x_grid;
    
    costmap_data_[index] = 100;
}

void CostmapNode::inflateObstacles() {
    std::vector<int8_t> inflated_costmap = costmap_data_;
        
    // figure out how many cells to inflate based on radius
    int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    
    // for each cell in the costmap
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int index = y * width_ + x;
            
            if (costmap_data_[index] == 100) {
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        // check bounds
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            // calculate Euclidean distance
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            
                            if (distance <= inflation_radius_ && distance > 0) {
                                int neighbor_index = ny * width_ + nx;
                                int cost = static_cast<int>(100 * (1.0 - distance / inflation_radius_));
                                
                                inflated_costmap[neighbor_index] = std::max(
                                    inflated_costmap[neighbor_index], 
                                    static_cast<int8_t>(cost));}}}}}}}
    costmap_data_ = inflated_costmap;
}

void CostmapNode::publishCostmap() {
    auto costmap_msg = nav_msgs::msg::OccupancyGrid();
    costmap_msg.header.stamp = this->now();
    costmap_msg.header.frame_id = "base_link";  
    // metadata
    costmap_msg.info.resolution = resolution_;
    costmap_msg.info.width = width_;
    costmap_msg.info.height = height_;
    
    costmap_msg.info.origin.position.x = -(width_ * resolution_) / 2.0;
    costmap_msg.info.origin.position.y = -(height_ * resolution_) / 2.0;
    costmap_msg.info.origin.position.z = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;
    
    // copy costmap data and publish
    costmap_msg.data = costmap_data_;
    costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}