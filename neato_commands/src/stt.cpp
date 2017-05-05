#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>

std_msgs::String command;

void audioCallback(const std_msgs::String::ConstPtr msg) {
  command = msg->data.c_str();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "speech_to_text");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<std_msgs::String>("audio_data", 1000, audioCallback);
  ros::Publisher pub = n.advertise<std_msgs::String>("interpreted_command", 1000)
  ros::Rate loop_rate(1);

  while(ros::ok()) {
    
    pub.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
