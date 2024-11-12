#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    
    

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)    //return bool to check if the robot has moved
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        resetPrevious(odometry);
        initialized_ = true;
    }
    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    ds_ = dx_ * dx_ + dy_ * dy_;
    dtheta_ = odometry.theta - previousPose_.theta;
    if(ds_ < min_dist_ && dtheta_ < min_theta_) return false;
    alpha = atan2(dy_,dx_) - previousPose_.theta;
    
    // for (int i = 0; i<1000;i++){
    //     mbot_lcm_msgs::pose2D_t newpose;
    //     newpose.x = previousPose_.x + (ds_ + kesi2(numberGenerator_)) * cos(previousPose_.theta + alpha + kesi1(numberGenerator_));
    //     newpose.y = previousPose_.y + (ds_ + kesi2(numberGenerator_)) * sin(previousPose_.theta + alpha + kesi1(numberGenerator_));
    //     newpose.theta = previousPose_.theta + dtheta_ + kesi1(numberGenerator_) + kesi3(numberGenerator_);


    // }
    

    return true;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    if(ds_ > min_dist_ && dtheta_ > min_theta_){    //moved
    std::normal_distribution<float> kesi1(0.0,k1_ * fabs(alpha));
    std::normal_distribution<float> kesi2(0.0 , k2_ * fabs(ds_));
    std::normal_distribution<float> kesi3(0.0 , k1_ * fabs(dtheta_ - alpha));
    newSample.pose.utime = utime_;
    newSample.pose.x = sample.pose.x + (ds_ + kesi2(numberGenerator_)) * cos(sample.pose.theta + alpha + kesi1(numberGenerator_));
    newSample.pose.y = sample.pose.y + (ds_ + kesi2(numberGenerator_)) * sin(sample.pose.theta + alpha + kesi1(numberGenerator_));
    newSample.pose.theta = sample.pose.theta + dtheta_ + kesi1(numberGenerator_) + kesi3(numberGenerator_);
    newSample.parent_pose = sample.pose;
    }
    else   //not moved, doing nothing...
    {
        newSample.pose.utime = utime_;
        newSample.pose = sample.pose;
    }

    return newSample;
}
