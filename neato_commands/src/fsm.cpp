#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

void command_callback(const std_msgs::String::ConstPtr msg) {
  switch(msg->data.c_str()) {
    case "follow":
      break;
    case "stop":
      break;
    case "dance":
      break;
  }
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<std_msgs::String>("actual_command", 1000, command_callback);
  ros::spin();
  return 0;
}