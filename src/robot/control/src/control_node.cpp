#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
    lookahead_dist = 1.0;
    linear_vel = 0.3;
    max_angular_vel = 1.0;
    goal_tolerance = 0.3;
    
    have_path = false;
    have_odom = false;
    
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlNode::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Control node started");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = *msg;
    have_path = !msg->poses.empty();
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    have_odom = true;
}

void ControlNode::controlLoop() {
    if(!have_path || !have_odom) {
        return;  
    }
    
    // check if reached goal
    if(current_path_.poses.empty()) {
        return;
    }
    
    auto goal = current_path_.poses.back();
    double dist_to_goal = getDistance(
        current_odom_.pose.pose.position.x,
        current_odom_.pose.pose.position.y,
        goal.pose.position.x,
        goal.pose.position.y
    );
    
    if(dist_to_goal < goal_tolerance) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_pub_->publish(cmd);
        return;
    }
    
    // find lookahead point
    auto target = findLookaheadPoint();
    
    // compute velocity command
    auto cmd = computeVelocity(target);
    cmd_pub_->publish(cmd);
}

geometry_msgs::msg::PoseStamped ControlNode::findLookaheadPoint() {
    double robot_x = current_odom_.pose.pose.position.x;
    double robot_y = current_odom_.pose.pose.position.y;
    
    // find point on path closest to lookahead distance
    for(size_t i = 0; i < current_path_.poses.size(); i++) {
        double dist = getDistance(
            robot_x, robot_y,
            current_path_.poses[i].pose.position.x,
            current_path_.poses[i].pose.position.y
        );
        
        if(dist >= lookahead_dist) {
            return current_path_.poses[i];
        }
    }
    
    // if no point far enough, return last point
    return current_path_.poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped& target) {
    geometry_msgs::msg::Twist cmd;
    
    // get robot position and heading
    double robot_x = current_odom_.pose.pose.position.x;
    double robot_y = current_odom_.pose.pose.position.y;
    double robot_yaw = getYaw(current_odom_.pose.pose.orientation);
    
    // calculate angle to target
    double dx = target.pose.position.x - robot_x;
    double dy = target.pose.position.y - robot_y;
    double target_angle = atan2(dy, dx);
    
    // calculate angle error
    double angle_error = normalizeAngle(target_angle - robot_yaw);
    
    // pure pursuit: calculate angular velocity based on angle to lookahead point
    double distance = getDistance(robot_x, robot_y, target.pose.position.x, target.pose.position.y);
    
    if(distance > 0.01) {
        // curvature = 2 * sin(alpha) / L
        double curvature = 2.0 * sin(angle_error) / distance;
        cmd.angular.z = curvature * linear_vel;
        
        // limit angular velocity
        if(cmd.angular.z > max_angular_vel) cmd.angular.z = max_angular_vel;
        if(cmd.angular.z < -max_angular_vel) cmd.angular.z = -max_angular_vel;
    } else {
        cmd.angular.z = 0;
    }
    
    // reduce speed when turning sharply
    if(fabs(angle_error) > 0.5) {
        cmd.linear.x = linear_vel * 0.5;
    } else {
        cmd.linear.x = linear_vel;
    }
    
    return cmd;
}

double ControlNode::getDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double ControlNode::normalizeAngle(double angle) {
    while(angle > M_PI) angle -= 2.0 * M_PI;
    while(angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double ControlNode::getYaw(const geometry_msgs::msg::Quaternion& quat) {
    // convert quaternion to yaw
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}