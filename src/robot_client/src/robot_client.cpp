#include "robot_client.hpp"

#include "robot_interface_topics.hpp"

RobotClient::RobotClient(const ros::NodeHandle &nodeHandle, const std::string &robotName,
                         const std::vector<geometry_msgs::PoseStamped> &goals)
    : _nodeHandle(nodeHandle), _robotName(robotName), _goals(goals), _currentGoal(0) {
    _positionSubscriber =
        _nodeHandle.subscribe(_robotName +
                                  RobotInterfaceTopics::Position,
                              QueueSize, &RobotClient::positionCallback, this);
}

void RobotClient::start() {
    ros::Rate rate(GoalCheckRate);
    requestMoveToNextGoal();

    while (ros::ok() && _currentGoal < _goals.size()) {
        if (goalReached()) {
            ROS_INFO_STREAM(_robotName << " reached goal: " << _currentGoal);
            _currentGoal++;
            requestMoveToNextGoal();
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM(_robotName << " finished all goals.");
}

bool RobotClient::goalReached() {
    auto position = getPosition().pose.position;
    double distance = std::hypot(position.x - _goals[_currentGoal].pose.position.x,
                                 position.y - _goals[_currentGoal].pose.position.y);
    ROS_INFO_STREAM("Distance to target is " << distance);
    return distance < GoalTolerance;
}

void RobotClient::requestMoveToNextGoal() {
    if (_currentGoal < _goals.size()) {
        robots_interface_server::MoveToPosition srv;
        srv.request.pose = _goals[_currentGoal];
        if (!ros::service::call(_robotName + RobotInterfaceTopics::MoveToPosition, srv)) {
            ROS_ERROR_STREAM("Failed to call service " << RobotInterfaceTopics::MoveToPosition);
            return;
        }
        ROS_INFO_STREAM(_robotName << " sending goal: " << _currentGoal);
    }
}

void RobotClient::positionCallback(const geometry_msgs::PoseStamped::ConstPtr &positionMsg) {
    ROS_INFO_STREAM(_robotName << " position is: {"
                               << positionMsg->pose.position.x << ", "
                               << positionMsg->pose.position.y << "}");
    std::lock_guard<std::mutex> lock(_positionMutex);
    _currentPosition = *positionMsg;
}

geometry_msgs::PoseStamped RobotClient::getPosition() {
    std::lock_guard<std::mutex> lock(_positionMutex);
    return _currentPosition;
}