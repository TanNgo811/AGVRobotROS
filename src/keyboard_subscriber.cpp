#include "ros/ros.h"

#include "std_msgs/String.h"

#include "avg_robot/Navigation.h"


void chatterCallback(const avg_robot::Navigation navi)
{

  ROS_INFO("receive direction: [%s]", navi.navigation.c_str());

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_robot_subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("input_keyboard", 1, chatterCallback);

  ros::spin();

  return 0;
}
