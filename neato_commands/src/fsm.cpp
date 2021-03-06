#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>

std_msgs::String command;

void setTwistVals(geometry_msgs::Twist msg, double speed, double spin) {
        msg.linear.x = speed;
        msg.angular.z = spin;
}

void commandCallback(const std_msgs::String::ConstPtr msg) {
  command = msg->data.c_str();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<std_msgs::String>("interpreted_command", 1000, commandCallback);
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000)
  ros::Rate loop_rate(1);

  geometry_msgs::Twist twist;

  while(ros::ok()) {
    switch(command) {
      case "go":
        setTwistVals(twist, 1, 0);
        break;
      case "stop":
        setTwistVals(twist, 0, 0);
        break;
      case "left":
        setTwistVals(twist, 0, 0.5);
        break;
      case "right":
        setTwistVals(twist, 0, -0.5);
        break;
      case "dance":
        double rand_speed = 2*double(rand())/double(RAND_MAX)-1;
        double rand_spin = 2*double(rand())/double(RAND_MAX)-1;
        setTwistVals(twist, rand_speed, rand_spin);
        break;
    }
    movement_pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
