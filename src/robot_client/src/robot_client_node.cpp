#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>

#include "robots_interface_server/GetPosition.h"
#include "robots_interface_server/MoveToPosition.h"

class GoalClient {
public:
  GoalClient(const ros::NodeHandle& nodeHandle, const std::string& robot_name,
             const std::vector<geometry_msgs::PoseStamped>& goals)
    : _nodeHangle(nodeHandle), _robotName(robot_name), _goals(goals), _currentGoal(0) {
    _goalPublisher = _nodeHangle.advertise<geometry_msgs::PoseStamped>(_robotName + "/move_to", 10);
  }

  void start() {
    ros::Rate rate(1);
    //sendNextGoal();
    requestMoveToNextGoal();

    while (ros::ok() && _currentGoal < _goals.size()) {
      if (goalReached()) {
        ROS_INFO_STREAM(_robotName << " reached goal: " << _currentGoal);
        _currentGoal++;
        //sendNextGoal();
        requestMoveToNextGoal();
      }
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO_STREAM(_robotName << " finished all goals.");
  }

private:
  bool goalReached() {
    robots_interface_server::GetPosition srv;
    if (!ros::service::call(_robotName + "/get_position", srv)) {
      ROS_ERROR_STREAM("Failed to call service /get_position");
      return false;
    }
    
    double distance = std::hypot(srv.response.pose.pose.position.x - _goals[_currentGoal].pose.position.x,
                                 srv.response.pose.pose.position.y - _goals[_currentGoal].pose.position.y);
    const double goal_tolerance = 0.1;
    ROS_INFO_STREAM(distance);
    return distance < goal_tolerance;
  }

  void sendNextGoal() {
    if (_currentGoal < _goals.size()) {
      _goalPublisher.publish(_goals[_currentGoal]);
      ROS_INFO_STREAM(_robotName << " sending goal: " << _currentGoal);
    }
  }

  void requestMoveToNextGoal() {
    if (_currentGoal < _goals.size()) {
      robots_interface_server::MoveToPosition srv;
      srv.request.pose = _goals[_currentGoal];
      if (!ros::service::call(_robotName + "/move_to_position", srv)) {
        ROS_ERROR_STREAM("Failed to call service /move_to_position");
        return;
      }
      ROS_INFO_STREAM(_robotName << " sending goal: " << _currentGoal);
    }
  }

  ros::NodeHandle _nodeHangle;
  ros::Publisher _goalPublisher;
  std::string _robotName;
  std::vector<geometry_msgs::PoseStamped> _goals;
  size_t _currentGoal;
};

std::vector<geometry_msgs::PoseStamped> createDummyGoals() {
  std::vector<geometry_msgs::PoseStamped> goals;

  geometry_msgs::PoseStamped goal1;
  goal1.header.frame_id = "map";
  goal1.header.stamp = ros::Time::now();
  goal1.pose.position.x = 1.0;
  goal1.pose.position.y = 1.0;
  goal1.pose.orientation.z = 0.0;
  goal1.pose.orientation.w = 1.0;
  goals.emplace_back(std::move(goal1));

  geometry_msgs::PoseStamped goal2;
  goal2.header.frame_id = "map";
  goal2.header.stamp = ros::Time::now();
  goal2.pose.position.x = 2.0;
  goal2.pose.position.y = 2.0;
  goal2.pose.orientation.z = 0.0;
  goal2.pose.orientation.w = 1.0;
  goals.emplace_back(std::move(goal2));

  geometry_msgs::PoseStamped goal3;
  goal3.header.frame_id = "map";
  goal3.header.stamp = ros::Time::now();
  goal3.pose.position.x = 3.0;
  goal3.pose.position.y = 3.0;
  goal3.pose.orientation.z = 0.0;
  goal3.pose.orientation.w = 1.0;
  goals.emplace_back(std::move(goal3));

  return goals;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_client_node");
  ros::NodeHandle nodeHandle;
  std::string robot_name = "robot1";

  GoalClient client(nodeHandle, robot_name, createDummyGoals());
  client.start();

  return 0;
}
