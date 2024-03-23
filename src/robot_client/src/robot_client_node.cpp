#include "robot_client.hpp"

geometry_msgs::PoseStamped createDummyPoseStamped(double x, double y) {
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = "map";
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose.position.x = x;
  poseStamped.pose.position.y = y;
  poseStamped.pose.orientation.z = 0.0;
  poseStamped.pose.orientation.w = 1.0;
  return poseStamped;
}

std::vector<geometry_msgs::PoseStamped> createDummyGoals() {
  return {
    createDummyPoseStamped(1.0f, 1.0f),
    createDummyPoseStamped(2.0f, 2.0f),
    createDummyPoseStamped(3.0f, 3.0f),
    createDummyPoseStamped(4.0f, 4.0f),
  };
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_client_node");
  ros::NodeHandle nodeHandle;
  std::string robotName = "robot1";

  RobotClient client(nodeHandle, robotName, createDummyGoals());
  client.start();

  return 0;
}
