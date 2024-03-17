#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <mutex>

#include "robots_interface_server/GetPosition.h"

class RobotInterface
{
public:
  RobotInterface(const ros::NodeHandle& nodeHandle, const std::string& robotName)
    : _nodeHandle(nodeHandle), _robotName(robotName) {
    _goalSubscriber = _nodeHandle.subscribe(_robotName + "/move_to", 10, &RobotInterface::goalCallback, this);
    _positionPublisher = _nodeHandle.advertise<nav_msgs::Odometry>(_robotName + "/current_position", 10);
    _positionService = _nodeHandle.advertiseService(_robotName + "/get_position", &RobotInterface::getPosition, this);
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
    ROS_INFO_STREAM(_robotName << " is moving to x: " << goal_msg->pose.position.x << ", y: " << goal_msg->pose.position.y);
    
    std::lock_guard<std::mutex> lock(_positionMutex);
    _currentPosition.pose.pose = goal_msg->pose;
    _positionPublisher.publish(_currentPosition);
  }

  bool getPosition(robots_interface_server::GetPosition::Request& request,
                   robots_interface_server::GetPosition::Response& response) {
    std::lock_guard<std::mutex> lock(_positionMutex);

    response.pose.header.stamp = ros::Time::now();
    response.pose.header.frame_id = _currentPosition.header.frame_id;
    response.pose.pose = _currentPosition.pose.pose;

    return true;
  }

private:
  ros::NodeHandle _nodeHandle;
  ros::Subscriber _goalSubscriber;
  ros::Publisher _positionPublisher;
  ros::ServiceServer _positionService;
  nav_msgs::Odometry _currentPosition;
  std::string _robotName;
  std::mutex _positionMutex;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_interface_node");
  ros::NodeHandle nodeHandle;

  RobotInterface robot1(nodeHandle, "robot1");
  RobotInterface robot2(nodeHandle, "robot2");

  ros::spin();

  return 0;
}
