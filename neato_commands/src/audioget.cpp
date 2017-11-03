#include <ros/ros.h>
#include "audio_common_msgs/AudioData.msg"

int Audio[] = {};

void AudioCallback(const audio_common_msgs::AudioData msg) {
	mp3_file = open('live.mp3','w')
  
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "audioget");
  ros::NodeHandle n; 
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe<std_msgs::String>("Audio", 1000, AudioCallback);
  ros::Publisher movement_pub = n.advertise<audio_common_msgs::AudioData>("SlicedAudio", 1000)
  ros::spin();

}