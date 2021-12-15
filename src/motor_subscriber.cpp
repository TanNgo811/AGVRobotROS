#include "ros/ros.h"

#include "std_msgs/String.h"

#include "avg_robot/MotorControl.h"

void chatterCallback(const avg_robot::MotorControl motor)
{

    ROS_INFO("receive direction: [%s]", motor.control.c_str());
    if (motor.control == "UP") {
        // Toc do > 0 thi dong co xoay theo chieu kim dong ho, Toc do < 0 thi nguoc lai
        ROS_INFO("Motor 1: [100] - Motor 2: [-100]");
    } else if (motor.control == "DOWN") {
        ROS_INFO("Motor 1: [-100] - Motor 2: [100]");
    } else if (motor.control == "RIGHT") {
        ROS_INFO("Motor 1: [-100] - Motor 2: [-100]");
    } else if (motor.control == "LEFT") {
        ROS_INFO("Motor 1: [100] - Motor 2: [100]");
    } else {
        ROS_INFO("Stop - Motor 1: [0] - Motor 2: [0]");
    } 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_control_subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("motor_control", 1, chatterCallback);

  ros::spin();

  return 0;
}
