#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "robots_interface_server/GetPosition.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_client_node");
  ros::NodeHandle nodeHandle;

  ros::ServiceClient positionClient = nodeHandle.serviceClient<robots_interface_server::GetPosition>("robot1/get_position");

  robots_interface_server::GetPosition srv;

  if (positionClient.call(srv)) {
    ROS_INFO_STREAM("Received pose from robot1: "
                    << "Position (x,y,z) = ("
                    << srv.response.pose.pose.position.x << ","
                    << srv.response.pose.pose.position.y << ","
                    << srv.response.pose.pose.position.z << ") "
                    << "Orientation (x,y,z,w) = ("
                    << srv.response.pose.pose.orientation.x << ","
                    << srv.response.pose.pose.orientation.y << ","
                    << srv.response.pose.pose.orientation.z << ","
                    << srv.response.pose.pose.orientation.w << ")");
  } else {
    ROS_ERROR("Failed to call service get_position");
  }

  return 0;
}