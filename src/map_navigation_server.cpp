#include "ros/ros.h"

#include "avg_robot/MapNavigation.h"

#include "Astar.cpp"

#include <string.h>


std::string path_finder(Astar a, Map warehouseMap, int startPoint, int endPoint);
   
   
bool path_finder_service(avg_robot::MapNavigation::Request  &req, avg_robot::MapNavigation::Response &res) {
    ROS_INFO("request: startPoint=%d, packagePoint=%d, endPoint=%d", req.startPoint, req.packagePoint, req.endPoint);

    Astar a;

    Map warehouseMap;

    std::string result_finder = "";

    result_finder = path_finder(a, warehouseMap, req.startPoint, req.packagePoint);

    result_finder += path_finder(a, warehouseMap, req.packagePoint, req.endPoint);
    
    res.navigationArray = result_finder;



    ROS_INFO("sending back response - Path finder: [%s]", res.navigationArray.c_str());
    return true;
}
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_navigation_server");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("map_navigation", path_finder_service);
    
    ROS_INFO("Nhap cac tham so [Start] [Package] [End] o Client");
    
    ros::spin();

    return 0;
}

std::string path_finder(Astar a, Map warehouseMap, int startPoint, int endPoint) {
    std::vector<int> mapArray {1, 1, 1, 1, 0, 0, 0,
                               0, 0, 0, 1, 1, 0, 0,
                               0, 0, 1, 1, 1, 1, 1,
                               1, 1, 1, 1, 1, 1, 0};
    
    warehouseMap.storeMap(mapArray);
    warehouseMap.displayMap();

    bool success = a.createNodeList(warehouseMap, startPoint-1, endPoint-1);

    if (success == true) {
        return a.planPath();
    } else {
        std::cout << "Enter a valid start and end point" << std::endl;
    }

    return "";

}
