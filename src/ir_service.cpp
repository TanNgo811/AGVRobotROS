#include "ros/ros.h"

#include "avg_robot/IRSignal.h"

#include <string.h>
   
bool avg(avg_robot::IRSignal::Request  &req, avg_robot::IRSignal::Response &res) {
    ROS_INFO("request: IR_1=%d, IR_2=%d, IR_3=%d, IR_4=%d, IR_5=%d", req.IR_1, req.IR_2, req.IR_3, req.IR_4, req.IR_5);

    if (req.IR_2 && req.IR_3 && req.IR_4) {
        if (req.IR_1 && req.IR_5) res.state = "Fork/Crossroad";
        else if (req.IR_1 && !req.IR_5) res.state = "Fork/LeftTurn";
        else if (req.IR_5 && !req.IR_1) res.state = "Fork/RightTurn";
        else res.state = "StraightLine";
    } else if (req.IR_2 && req.IR_3 && !req.IR_4) {
        res.state = "RightSkew";
    } else if (!req.IR_2 && req.IR_3 && req.IR_4) {
        res.state = "LeftSkew";
    } else if (!req.IR_3) {
        res.state = "MissedLine";
    } else {
        res.state = "ERR";
    }

    ROS_INFO("sending back response - Direction: [%s]", res.state.c_str());
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