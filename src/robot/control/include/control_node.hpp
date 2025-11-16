#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "control_core.hpp"
#include <cmath>
#include <vector>

class ControlNode : public rclcpp::Node {
public:
    ControlNode();
    
private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    
    geometry_msgs::msg::PoseStamped findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target);
    double getDistance(double x1, double y1, double x2, double y2);
    double normalizeAngle(double angle);
    double getYaw(const geometry_msgs::msg::Quaternion& quat);
    
    robot::ControlCore control_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry current_odom_;
    
    bool have_path;
    bool have_odom;
    
    double lookahead_dist;
    double linear_vel;
    double max_angular_vel;
    double goal_tolerance;
};

#endif