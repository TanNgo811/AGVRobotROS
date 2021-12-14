#include "ros/ros.h"

#include "avg_robot/IRSignal.h"

#include <string.h>
   
bool avg(avg_robot::IRSignal::Request  &req, avg_robot::IRSignal::Response &res) {
    ROS_INFO("request: IR_1=%ld, IR_2=%ld, IR_3=%ld, IR_4=%ld, IR_5=%ld", req.IR_1, req.IR_2, req.IR_3, req.IR_4, req.IR_5);

    if (req.IR_1 == 0 && req.IR_5 == 0 && req.IR_2 == 1 && req.IR_3 == 1 && req.IR_4 == 1) {
        res.direction = "forward";
    } else if (req.IR_1 == 1 && req.IR_5 == 1) {
        res.direction = "forward";
    } else if (req.IR_1 == 1) {
        res.direction = "left";
    } else if (req.IR_5 == 1) {
        res.direction = "right";
    } else {
        res.direction = "forward";
    }

    ROS_INFO("sending back response - Direction: [%s]", res.direction.c_str());
    return true;
}
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ir_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("ir_signal", avg);
    ROS_INFO("Ready to recieve IR.");
    ros::spin();

    return 0;
}