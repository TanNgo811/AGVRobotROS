
#include "ros/ros.h"

#include "std_msgs/String.h"

#include "avg_robot/Distance.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_publisher");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<avg_robot::Distance>("robot_distance", 1000);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    avg_robot::Distance msg;
   
    msg.distance = atoi(argv[1]);

    ROS_INFO("%ld", msg.distance);

    chatter_pub.publish(msg);
    
    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
