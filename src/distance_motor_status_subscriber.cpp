

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "avg_robot/Distance.h"

#include "avg_robot/MotorWarning.h"

bool enableMotor = true;

void distanceCallback(const avg_robot::Distance msg)
{
    ROS_INFO("Received distance: [%ld]", msg.distance);

    if (msg.distance <= 10) {
        ROS_INFO("STOP!");
        enableMotor = false;
    } else {
        ROS_INFO("CONTINUE!");
        enableMotor = true;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_motor_status_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("robot_distance", 1000, distanceCallback);

    ros::Publisher chatter_pub = n.advertise<avg_robot::MotorWarning>("motor_warning", 1000);

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        avg_robot::MotorWarning warningMsg;

        warningMsg.enable = enableMotor;

        ROS_INFO("send Warning to motor [%d]", warningMsg.enable);

        chatter_pub.publish(warningMsg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    
    

    ros::spin();

    return 0;
}
