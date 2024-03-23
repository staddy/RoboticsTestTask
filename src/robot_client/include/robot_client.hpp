#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <mutex>

#include "robots_interface_server/MoveToPosition.h"

class RobotClient {
public:
    RobotClient(const ros::NodeHandle &nodeHandle, const std::string &robotName,
                const std::vector<geometry_msgs::PoseStamped> &goals);

    void start();

private:
    inline static constexpr const double GoalTolerance = 0.1f;
    inline static constexpr const size_t GoalCheckRate = 4;
    inline static constexpr const size_t QueueSize = 10;

    bool goalReached();
    void requestMoveToNextGoal();
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &positionMsg);
    geometry_msgs::PoseStamped getPosition();

    ros::NodeHandle _nodeHandle;
    ros::Subscriber _positionSubscriber;
    std::string _robotName;
    std::vector<geometry_msgs::PoseStamped> _goals;
    geometry_msgs::PoseStamped _currentPosition;
    size_t _currentGoal;
    std::mutex _positionMutex;
};