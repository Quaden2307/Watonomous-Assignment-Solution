#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot {

class PlannerCore {
public:
    PlannerCore(const rclcpp::Logger& logger);
    ~PlannerCore();

private:
    rclcpp::Logger logger_;
};

}

#endif