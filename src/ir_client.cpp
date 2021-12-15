   #include "ros/ros.h"
   
   #include "avg_robot/IRSignal.h"
   
   #include <string.h>
   
   #include <cstdlib>
   
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "ir_client");
     if (argc != 6)
      {
       ROS_INFO("Nhap tin hieu cua 5 den hong ngoai");
       return 1;
     }
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<avg_robot::IRSignal>("ir_signal");
     avg_robot::IRSignal srv;
     srv.request.IR_1 = atoi(argv[1]);
     srv.request.IR_2 = atoi(argv[2]);
     srv.request.IR_3 = atoi(argv[3]);
     srv.request.IR_4 = atoi(argv[4]);
     srv.request.IR_5 = atoi(argv[5]);
     if (client.call(srv))
     {
       ROS_INFO("Direction: %s", srv.response.state.c_str());
     }
     else
     {
       ROS_ERROR("Failed to call IR service");
       return 1;
     }
   
     return 0;
  }