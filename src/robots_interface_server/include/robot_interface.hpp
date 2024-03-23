#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>

#include "robots_interface_server/MoveToPosition.h"

class RobotInterface {
public:
    RobotInterface(const ros::NodeHandle &, const std::string &);

private:
    inline static constexpr const double PositionPublishPeriod = 0.1f;
    inline static constexpr const size_t QueueSize = 10;

    bool moveToPosition(robots_interface_server::MoveToPosition::Request &request,
                        robots_interface_server::MoveToPosition::Response &response);
    void publishPosition(const ros::TimerEvent &);

    ros::NodeHandle _nodeHandle;
    ros::Publisher _positionPublisher;
    ros::ServiceServer _moveToPositionService;
    geometry_msgs::PoseStamped _currentPosition;
    std::string _robotName;
    std::mutex _positionMutex;
    ros::Timer _publishTimer;
};