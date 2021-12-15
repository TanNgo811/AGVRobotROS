

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "avg_robot/Distance.h"

int count = 0;

void distanceCallback(const avg_robot::Distance msg)
{
    ROS_INFO("Received distance: [%d] in [%d]", msg.distance, count);

    if (msg.distance <= 10) {
        count++;
    } else {
        count = 0;
    }

    if (count >= 10) {
            ROS_INFO("Canh bao! Khoang cach gan!");
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_warning_subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("robot_distance", 1000, distanceCallback);

  ros::spin();

  return 0;
}
