#include "robot_interface.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "position_control_node");
  ros::NodeHandle nodeHandle;

  RobotInterface robot1(nodeHandle, "robot1");
  RobotInterface robot2(nodeHandle, "robot2");

  ros::spin();

  return 0;
}
