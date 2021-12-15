   #include "ros/ros.h"
   
   #include "avg_robot/MapNavigation.h"
   
   #include <string.h>
   
   #include <cstdlib>

   #include "Astar.cpp"
   
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "map_navigation_client");

     Map warehouseMap;

     std::vector<int> mapArray {1, 1, 1, 1, 0, 0, 0,
                               0, 0, 0, 1, 1, 0, 0,
                               0, 0, 1, 1, 1, 1, 1,
                               1, 1, 1, 1, 1, 1, 0};

     
     if (argc == 2) {
      warehouseMap.storeMap(mapArray);
      warehouseMap.displayMap();
     }

     if (argc != 4)
      {
       ROS_INFO("Nhap lai cac tham so [Start] [Package] [End]");
       return 1;
     }
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<avg_robot::MapNavigation>("map_navigation");
     avg_robot::MapNavigation srv;
     srv.request.startPoint = atoi(argv[1]);
     srv.request.packagePoint = atoi(argv[2]);
     srv.request.endPoint = atoi(argv[3]);
     if (client.call(srv))
     {
       ROS_INFO("Direction Array: %s", srv.response.navigationArray.c_str());
     }
     else
     {
       ROS_ERROR("Failed to call path finder service");
       return 1;
     }
   
     return 0;
  }