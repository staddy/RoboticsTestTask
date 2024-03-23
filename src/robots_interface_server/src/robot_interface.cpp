#include "robot_interface.hpp"

#include "robot_interface_topics.hpp"

RobotInterface::RobotInterface(const ros::NodeHandle &nodeHandle, const std::string &robotName)
    : _nodeHandle(nodeHandle), _robotName(robotName) {
    _positionPublisher =
        _nodeHandle.advertise<geometry_msgs::PoseStamped>(_robotName +
                                                              RobotInterfaceTopics::Position,
                                                          QueueSize);
    _moveToPositionService =
        _nodeHandle.advertiseService(_robotName +
                                         RobotInterfaceTopics::MoveToPosition,
                                     &RobotInterface::moveToPosition, this);
    _publishTimer = _nodeHandle.createTimer(ros::Duration(PositionPublishPeriod),
                                            &RobotInterface::publishPosition, this);
}

void RobotInterface::publishPosition(const ros::TimerEvent &) {
    std::lock_guard<std::mutex> lock(_positionMutex);
    _positionPublisher.publish(_currentPosition);
}

bool RobotInterface::moveToPosition(robots_interface_server::MoveToPosition::Request &request,
                    robots_interface_server::MoveToPosition::Response &response) {
  std::lock_guard<std::mutex> lock(_positionMutex);
  _currentPosition = request.pose;
  return true;
}