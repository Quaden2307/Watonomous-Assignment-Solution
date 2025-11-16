#include "map_memory_node.hpp"
#include <algorithm>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    move_threshold = 1.5;  // Assignment specifies 1.5 meters
    map_size = 400;
    map_res = 0.2;  // IMPORTANT: Coarser than costmap (0.1m) to avoid holes
    
    last_x = 0.0;
    last_y = 0.0;
    curr_x = 0.0;
    curr_y = 0.0;
    got_costmap = false;
    need_update = false;
    
    // setup global map
    global_map_.info.width = map_size;
    global_map_.info.height = map_size;
    global_map_.info.resolution = map_res;
    global_map_.data.resize(map_size * map_size, -1);  
    global_map_.header.frame_id = "odom";
    
    //set origin to center
    global_map_.info.origin.position.x = -(map_size * map_res) / 2.0;
    global_map_.info.origin.position.y = -(map_size * map_res) / 2.0;
    global_map_.info.origin.orientation.w = 1.0;
    
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),  // CHANGED: from 1 second to 500ms for faster updates
        std::bind(&MapMemoryNode::updateMap, this));
    
    RCLCPP_INFO(this->get_logger(), "Map memory started");
    
    publishCurrentMap();
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_costmap_ = *msg;
    
    // ADDED: Force first update when we get the first costmap
    if(!got_costmap) {
        need_update = true;
        RCLCPP_INFO(this->get_logger(), "First costmap received - forcing update");
    }
    
    got_costmap = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr_x = msg->pose.pose.position.x;
    curr_y = msg->pose.pose.position.y;
    
    double dist = distanceMoved();
    
    // ADDED: Debug logging
    if(dist >= move_threshold) {
        need_update = true;
        last_x = curr_x;
        last_y = curr_y;
        RCLCPP_INFO(this->get_logger(), "Moved %.2fm - triggering map update", dist);
    }
}

void MapMemoryNode::updateMap() {
    if(need_update && got_costmap) {
        RCLCPP_INFO(this->get_logger(), "Integrating costmap into map");
        integrateCostmap();
        publishCurrentMap();
        need_update = false;
    }
}

double MapMemoryNode::distanceMoved() {
    double dx = curr_x - last_x;
    double dy = curr_y - last_y;
    return sqrt(dx*dx + dy*dy);
}

void MapMemoryNode::integrateCostmap() {
    double robot_x = curr_x;
    double robot_y = curr_y;
    
    int cells_updated = 0;  // ADDED: count cells for debugging
    
    for(int cy = 0; cy < (int)current_costmap_.info.height; cy++) {
        for(int cx = 0; cx < (int)current_costmap_.info.width; cx++) {
            int costmap_idx = cy * current_costmap_.info.width + cx;
            int8_t cost_value = current_costmap_.data[costmap_idx];
            
            if(cost_value < 0) continue;
            
            double cell_x = current_costmap_.info.origin.position.x + cx * current_costmap_.info.resolution;
            double cell_y = current_costmap_.info.origin.position.y + cy * current_costmap_.info.resolution;
            
            double world_x = robot_x + cell_x;
            double world_y = robot_y + cell_y;
            
            int map_x = (int)((world_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
            int map_y = (int)((world_y - global_map_.info.origin.position.y) / global_map_.info.resolution);
            
            if(map_x >= 0 && map_x < map_size && map_y >= 0 && map_y < map_size) {
                int map_idx = map_y * map_size + map_x;
                global_map_.data[map_idx] = cost_value;
                cells_updated++;
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Updated %d cells in global map", cells_updated);
}

void MapMemoryNode::publishCurrentMap() {
    global_map_.header.stamp = this->now();
    map_pub_->publish(global_map_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}