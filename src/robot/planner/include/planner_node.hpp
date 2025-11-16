#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planner_core.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

// helper structs for A*
struct GridCell {
    int x;
    int y;
    GridCell(int xx, int yy) : x(xx), y(yy) {}
    GridCell() : x(0), y(0) {}
    bool operator==(const GridCell &other) const {
        return (x == other.x && y == other.y);
    }
};

struct CellHash {
    std::size_t operator()(const GridCell &cell) const {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

struct AStarNode {
    GridCell cell;
    double f_score;
    AStarNode(GridCell c, double f) : cell(c), f_score(f) {}
    bool operator>(const AStarNode &other) const {
        return f_score > other.f_score;
    }
};

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();
    
private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void plannerTimer();
    
    std::vector<GridCell> astar(GridCell start, GridCell goal);
    double heuristic(GridCell a, GridCell b);
    std::vector<GridCell> getNeighbors(GridCell cell);
    bool isValid(GridCell cell);
    void worldToGrid(double x, double y, int& gx, int& gy);
    void gridToWorld(int gx, int gy, double& x, double& y);
    bool goalReached();
    
    robot::PlannerCore planner_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_point_;
    nav_msgs::msg::Odometry current_odom_;
    
    bool have_map;
    bool have_goal;
    bool have_odom;
    
    double goal_tolerance;
};

#endif