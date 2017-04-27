#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

std_msgs::String command;

void commandCallback(const std_msgs::String::ConstPtr msg) {
  command = msg->data.c_str();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<std_msgs::String>("actualCommand", 1000, command_callback);
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000)
  ros::Rate loop_rate(1);

  while(ros::ok()) {
    ros::spinOnce();
    switch(command) {
      case "follow":
        break;
      case "stop":
        break;
      case "dance":
        break;
    }
    loop_rate.sleep();
  }

  return 0;
}
