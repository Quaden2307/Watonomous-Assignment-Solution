#include "planner_node.hpp"
#include <algorithm>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    have_map = false;
    have_goal = false;
    have_odom = false;
    goal_tolerance = 0.5;
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10,
        std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::plannerTimer, this));
    
    RCLCPP_INFO(this->get_logger(), "Planner started");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    have_map = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_point_ = *msg;
    have_goal = true;
    RCLCPP_INFO(this->get_logger(), "Got new goal: (%.2f, %.2f)", 
                goal_point_.point.x, goal_point_.point.y);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    have_odom = true;
}

void PlannerNode::plannerTimer() {
    if(!have_map || !have_goal || !have_odom) {
        return;
    }
    
    if(goalReached()) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        have_goal = false;
        return;
    }
    
    // get start and goal in grid coords
    int start_x, start_y;
    worldToGrid(current_odom_.pose.pose.position.x, 
                current_odom_.pose.pose.position.y, 
                start_x, start_y);
    
    int goal_x, goal_y;
    worldToGrid(goal_point_.point.x, goal_point_.point.y, goal_x, goal_y);
    
    GridCell start(start_x, start_y);
    GridCell goal(goal_x, goal_y);
    
    // run A*
    std::vector<GridCell> path_cells = astar(start, goal);
    
    if(path_cells.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found!");
        return;
    }
    
    // convert to Path message
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "odom";
    
    for(const auto& cell : path_cells) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        
        double wx, wy;
        gridToWorld(cell.x, cell.y, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;
        
        path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);
}

bool PlannerNode::goalReached() {
    double dx = goal_point_.point.x - current_odom_.pose.pose.position.x;
    double dy = goal_point_.point.y - current_odom_.pose.pose.position.y;
    return sqrt(dx*dx + dy*dy) < goal_tolerance;
}

std::vector<GridCell> PlannerNode::astar(GridCell start, GridCell goal) {
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_map<GridCell, double, CellHash> g_score;
    std::unordered_map<GridCell, GridCell, CellHash> came_from;
    
    g_score[start] = 0;
    open_set.push(AStarNode(start, heuristic(start, goal)));
    
    while(!open_set.empty()) {
        GridCell current = open_set.top().cell;
        open_set.pop();
        
        if(current == goal) {
            // reconstruct path
            std::vector<GridCell> path;
            GridCell step = goal;
            while(!(step == start)) {
                path.push_back(step);
                step = came_from[step];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        for(const auto& neighbor : getNeighbors(current)) {
            if(!isValid(neighbor)) continue;
            
            double tentative_g = g_score[current] + 1.0;
            
            if(g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal);
                open_set.push(AStarNode(neighbor, f));
            }
        }
    }
    
    return std::vector<GridCell>();  // no path found
}

double PlannerNode::heuristic(GridCell a, GridCell b) {
    // euclidean distance
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

std::vector<GridCell> PlannerNode::getNeighbors(GridCell cell) {
    std::vector<GridCell> neighbors;
    // 8-connected grid
    for(int dx = -1; dx <= 1; dx++) {
        for(int dy = -1; dy <= 1; dy++) {
            if(dx == 0 && dy == 0) continue;
            neighbors.push_back(GridCell(cell.x + dx, cell.y + dy));
        }
    }
    return neighbors;
}

bool PlannerNode::isValid(GridCell cell) {
    if(cell.x < 0 || cell.x >= (int)current_map_.info.width ||
       cell.y < 0 || cell.y >= (int)current_map_.info.height) {
        return false;
    }
    
    int idx = cell.y * current_map_.info.width + cell.x;
    int8_t cost = current_map_.data[idx];
    
    if(cost > 70) {
        return false;
    }
    
    return true;
}

void PlannerNode::worldToGrid(double x, double y, int& gx, int& gy) {
    gx = (int)((x - current_map_.info.origin.position.x) / current_map_.info.resolution);
    gy = (int)((y - current_map_.info.origin.position.y) / current_map_.info.resolution);
}

void PlannerNode::gridToWorld(int gx, int gy, double& x, double& y) {
    x = current_map_.info.origin.position.x + (gx + 0.5) * current_map_.info.resolution;
    y = current_map_.info.origin.position.y + (gy + 0.5) * current_map_.info.resolution;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}