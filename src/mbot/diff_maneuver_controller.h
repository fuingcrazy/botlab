#ifndef MANEUVERS_CONTROLLER_BASE_
#define MANEUVERS_CONTROLLER_BASE_

#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/twist2D_t.hpp>
#include <Eigen/Dense>

typedef Eigen::Matrix<double,3,3> mat3x3;
typedef Eigen::Matrix<double,3,2> mat3x2;
typedef Eigen::Matrix<double,2,2> mat2x2;


class ManeuverControllerBase
{
public:
    ManeuverControllerBase() = default;

    virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) = 0;
    virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose = false) = 0;
};

#endif