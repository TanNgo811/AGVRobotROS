
#include <iostream>
#include <memory>
#include <list>
#include <vector>
#include <string>
#include "Astar.cpp"

#define debug

int main() {
    Astar a;
    Map warehouseMap;
    #ifdef debug
    std::vector<int> mapArray {1, 1, 1, 1, 0, 0, 0,
                               0, 0, 0, 1, 1, 0, 0,
                               0, 0, 1, 1, 1, 1, 1,
                               1, 1, 1, 1, 1, 1, 0};
    #endif
    int startPt, endPt;
    int userChoice;
    warehouseMap.storeMap(mapArray);
    warehouseMap.displayMap();
    std::cout << "The numbers in the map indicate the node index ";
    std::cout << "and \"X\" indicate obstacles. Enter the ";
    std::cout << "node index of the Start and End points: ";
    std::cout << std::endl;
    std::cin >> startPt >> endPt;
    bool success = a.createNodeList(warehouseMap, startPt-1, endPt-1);
    std::string result = a.planPath();

    std::list<Layoutnodes> directions = a.GetCloseList();

    std::list<string> dir_string ;



    for (auto i : directions) {
        std::cout << i.getIndex() + 1 <<std::endl;
        std::cout << i.getRowIndex()<<" "<< i.getColumnIndex() <<std::endl;
    }


    return 0;
}
