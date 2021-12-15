
#include <stdio.h>
#include <math.h>
#include <typeinfo>
#include <vector>
#include <list>
#include "Layoutnodes.cpp"
#include "Map.cpp"
#include <iostream>
#include <memory>

#define OBSTACLE 9999

class Astar {
 private:
    int startPoint;
    int endPoint;
    int mapColumn;
    int mapRow;
    std::list<Layoutnodes> openList;
    std::list<Layoutnodes> closedList;
    std::vector<Layoutnodes> nodeList;
    std::vector<double> neighbourNodes;
    std::list<Layoutnodes> path;
   //  cv::Mat drawing;

 public:
    /**
     * @brief constructor Astar
     * @param none
     * @return none
     * initializes values of startPoint, endPoint, mapColumn
     * and mapRow. 
     */
    Astar() {
    startPoint = 0;
    endPoint = 0;
    mapColumn = 0;
    mapRow = 0;
    }

    std::list<Layoutnodes>GetCloseList();

    /**
     * @brief Function SetStartPoint
     * @param startIndex of type int
     * @return none
     * The following function initializes the values of user defined
     * start point in the map.
     */
    void setStartPoint(int startIndex);
    /**
     * @brief Function SetEndPoint
     * @param endIndex of type int
     * @return none
     * The following function initializes the values of user defined
     * end(goal) point in the map.
     */
    void setEndPoint(int endIndex);
    /**
     * @brief Function calcPathCost
     * @param id of type int
     * @return none
     * calculates path cost between current node and parent node.
     */
    void calcPathCost(int id);
    /**
     * @brief Function calcHeuristicCost
     * @param node of type int
     * @param goal of type Layoutnodes
     * @return none
     * The following function calculates the straight line
     * cost between current node and end(goal) point.
     */
    void calcHeuristicCost(int node, Layoutnodes goal);
    /**
     * @brief Function createNodeList
     * @param warehouseLayout of type Map
     * @param startPt of type int
     * @param endPt of type int
     * @return bool (true or false)
     * The following function creates a list of possible nodes
     * (neglecting obstacles) for the given Map.
     */
    bool createNodeList(Map warehouseLayout, int startPt, int endPt);
    /**
     * @brief Function plaanPath
     * @param none
     * @return shortestPathLength of type int
     * The following function implements the A* algorithm and
     * identifies the shortest path.
     */
    std::string planPath();
    /**
     * @brief Function identifyNode
     * @param x of type int
     * @param y of type int
     * @return index of type int
     * The following function identifies the node index from the
     * row and column values of the node.
     */
    int identifyNode(int x, int y);
    /**
     * @brief Function inOpenList
     * @param id of type int
     * @return bool value
     * The following function returns true if a node of the given
     * is in the openList, else returns false.
     */
    bool inOpenList(int id);
    /**
     * @brief Function inClosedList
     * @param id of type int
     * @return bool value
     * The following function returns true if a node of the given
     * is in the closeList, else returns false.
     */
    bool inClosedList(int id);
    /**
     * @brief Function displayMap
     * @param none
     * @return display of type Mat (OpenCV Matrix)
     * The following function returns true if a node of the given
     * is in the closeList, else returns false.
     */
   //  cv::Mat displayMap();
    /**
     * @brief destructor Astar
     * @param none
     * @return none 
     */
    ~Astar() {}
};
   /**
     * @brief Function priority
     * @param reference to node1 of type Layoutnodes
     * @param reference to node2 of type Layoutnodes
     * @return bool value
     * The following function returns true if a node1 cost is less
     * than node2 cost, else false.
     */
    bool priority(Layoutnodes &node1, Layoutnodes &node2);


std::list<Layoutnodes> Astar::GetCloseList() {
    return Astar::closedList;
}

// #define testing
bool Astar::createNodeList(Map warehouseLayout, int startPt, int endPt) {
    std::vector<int> map = warehouseLayout.getMap();
    Layoutnodes node;
    mapColumn = warehouseLayout.returnColumn();
    mapRow = warehouseLayout.returnRow();
    int index = 0;
    for (int i = 0; i < mapRow; i++) {
        for (int j = 0; j < mapColumn; j++) {
            if (map[i*mapColumn + j] == 1) {
                node.setNodeIndex(index++, j, i);
                nodeList.emplace_back(node);
            }
        }
    }
    #ifdef testing
    for (int i =0; i < index; i++) {
        nodeList[i].output();
    }
    #endif
    if (startPt >= 0 && endPt >= 0 && (unsigned)startPt < nodeList.size() && (unsigned)endPt < nodeList.size()) {
        setStartPoint(startPt);
        setEndPoint(endPt);
        return true;
    } else {
        return false;
    }
}

void Astar::setStartPoint(int startIndex) {
    startPoint = startIndex;
    // startPoint = 2;
}

void Astar::setEndPoint(int endIndex) {
    endPoint = endIndex;
    // endPoint = 12;
}

std::string Astar::planPath() {
    nodeList[startPoint].setHeuristicCost(1);
    nodeList[startPoint].setPathCost(0);
    nodeList[startPoint].setTotalCost();

    for (auto a : nodeList) {
        calcHeuristicCost(a.getIndex(), nodeList[endPoint]);
    }

    openList.emplace_back(nodeList[startPoint]);
    #ifdef testing
    for (auto i : openList) {
        std::cout << "cost" << i.getCost() << std::endl;
    }
    // std::cout << "openList: " << openList.size() << std::endl;
    #endif

    Map map;
    auto directions = map.returnDirection();
    int finalFoundFlag = 0;
    while (!openList.empty()) {
        openList.sort(priority);
        Layoutnodes currentNode = openList.front();
        closedList.emplace_back(currentNode);
        // std::cout << "after transfer: " << currentNode.getCost() << std::endl;
        openList.pop_front();

        if (currentNode.getIndex() == nodeList[endPoint].getIndex()) {
            // std::cout << "final found";
            finalFoundFlag = 1;
            #ifdef testing
            for (auto i : closedList) {
                std::cout << "\n Node Index: "<< i.getIndex() << " hcost: " << i.returnHCost() << "  totalcost: " << i.getCost() << " Parent: " << i.getParentIndex();
            }
            #endif
            break;
        } else {
        // neighbour
        int cRow = currentNode.getRowIndex();
        int cCol = currentNode.getColumnIndex();
        // std::cout << "\ncrow " << cRow << "\tccol " << cCol;
        std::vector<int> neighbourID;
        for (int i = 0; i < 4; i++) {
            int x = directions[2*i] + cRow;
            int y = directions[2*i +1] + cCol;
            // std::cout << "\nx: " << x << "y: " << y << std::endl;
            if (x < 0 || y < 0 || x > 3 || y > 6) {
            continue;
            } else {
                int id = identifyNode(x, y);
                if (id != OBSTACLE) {
                    bool closed = inClosedList(id);
                    if (closed == true) {
                        continue;
                    } else {
                        bool open1 = inOpenList(id);
                        if (open1 == false) {
                            neighbourID.emplace_back(id);
                        }
                    }
                }
            }
        }
        for (auto i : neighbourID) {
            nodeList[i].setParentIndex(currentNode.getIndex());
            nodeList[i].setPathCost(1);
            nodeList[i].setTotalCost();
            openList.push_back(nodeList[i]);
            #ifdef testing
            std::cout << "\nThe openlist size is: " << openList.size();
            std::cout << "\nThe closedlist size is: " << closedList.size();
            #endif
        }
        }
    }
    
    // #ifdef testing

    std::string path = "";

    for (auto i : closedList) {
        // std::cout << ", " << i.getIndex() + 1;
        path += " " + std::to_string(i.getIndex() + 1);
    }

    return path; 
    // #endif
    // cv::Mat bufferMat = displayMap();
    // int shortestPathLength = 0;
    // if (finalFoundFlag == 0) {
    //     shortestPathLength = -1;
    // } else {
    //     shortestPathLength = (closedList.size()-1);
    // }
}

bool Astar::inOpenList(int id) {
    int openFlag = 0;
    for (auto l : openList) {
        if (l.getIndex() == id) {
            if (l.getCost() > nodeList[id].getCost()) {
                l.setCost(nodeList[id].getCost());
            }
            openFlag = 1;
        }
    }
    if (openFlag == 1) {
        return true;
    } else {
        return false;
    }
}

int Astar::identifyNode(int x, int y) {
    int found = 0;
    int index = 0;
    for (auto n : nodeList) {
        // std::cout<<"\n row col"<<n.getRowIndex()<<" "<<n.getColumnIndex();
        if (n.getRowIndex() == x && n.getColumnIndex() == y) {
            found = 1;
            index = n.getIndex();
        }
    }
    if (found == 1) {
        return index;
    } else {
        return OBSTACLE;
    }
}

bool Astar::inClosedList(int id) {
    int closedFlag = 0;
    for (auto l : closedList) {
        if (l.getIndex() == id)
           closedFlag = 1;
    }
    if (closedFlag == 1)
        return true;
    else
        return false;
}


void Astar::calcHeuristicCost(int node, Layoutnodes goal) {
    int x1 = nodeList[node].getColumnIndex();
    int y1 = nodeList[node].getRowIndex();
    int x2 = goal.getColumnIndex();
    int y2 = goal.getRowIndex();
    double distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    nodeList[node].setHeuristicCost(distance);
}

// cv::Mat Astar::displayMap() {
//     int sqW = 50;
//     int iSqW = 45;
//     int xLength = mapColumn*sqW, yLength = mapRow*sqW+40;
//     drawing = Mat::zeros(Size(xLength, yLength), CV_8UC3);
//     for (int i = 0; i < mapRow; i++) {
//         for (int j = 0; j < mapColumn; j++) {
//             int pathFlag = 0;
//             int blockFlag = 1;
//             int firstPt = 0, lastPt = 0;
//             for (auto n : path) {
//                 if (n.getRowIndex() == i && n.getColumnIndex() == j) {
//                     pathFlag = 1;
//                     if (n.getIndex() == startPoint)
//                        firstPt = 1;
//                     else if (n.getIndex() == endPoint)
//                        lastPt = 1;
//                 }
//             }
//             for (auto n : nodeList) {
//                 if (n.getRowIndex() == i && n.getColumnIndex() == j) {
//                     blockFlag = 0;
//                 }
//             }
//             if (pathFlag == 1) {
//                 rectangle(drawing, Point(j*sqW, i*sqW), Point(j*sqW+sqW, i*sqW+sqW), Scalar(255, 255, 255), 1, CV_AA, 0);
//                 if (firstPt == 1)
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(0, 255, 255), -1, CV_AA, 0);
//                 else if (lastPt == 1)
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(0, 0, 255), -1, CV_AA, 0);
//                 else
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(0, 255, 0), -1, CV_AA, 0);
//             } else if (blockFlag == 1) {
//                 rectangle(drawing, Point(j*sqW, i*sqW), Point(j*sqW+sqW, i*sqW+sqW), Scalar(255, 255, 255), 1, CV_AA, 0);
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(128, 128, 128), -1, CV_AA, 0);
//             } else {
//                 rectangle(drawing, Point(j*sqW, i*sqW), Point(j*sqW+sqW, i*sqW+sqW), Scalar(255, 255, 255), 1, CV_AA, 0);
//               }
//        }
//     }
//     rectangle(drawing, Point(10, yLength-35), Point(20, yLength-25), Scalar(0, 255, 255), -1, CV_AA, 0);
//     putText(drawing, "- Start Point", Point(25, yLength-25), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
//     rectangle(drawing, Point(100, yLength-35), Point(110, yLength-25), Scalar(0, 0, 255), -1, CV_AA, 0);
//     putText(drawing, "- End Point", Point(125, yLength-25), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
//     rectangle(drawing, Point(10, yLength-15), Point(20, yLength-5), Scalar(0, 255, 0), -1, CV_AA, 0);
//     putText(drawing, "- Path", Point(25, yLength-10), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
//     rectangle(drawing, Point(100, yLength-15), Point(110, yLength-5), Scalar(128, 128, 128), -1, CV_AA, 0);
//     putText(drawing, "- Obstacle", Point(125, yLength-10), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
// return drawing;
// }

bool priority(Layoutnodes &node1, Layoutnodes &node2) {
    if (node1.getCost() < node2.getCost())
        return true;
    else
        return false;
}
