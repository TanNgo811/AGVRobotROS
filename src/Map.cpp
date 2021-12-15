
#include <vector>
#include <string>
#pragma once
#include <string.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Map {
 private:
    std::vector<int> currentMap;
    int column = 7;
    int row = 4;
    int moveDirection[8] = {-1, 0,    // top
                             0, 1,    // right
                             1, 0,    // bottom
                             0, -1};  // left

 public:
    /**
     * @brief constructor for Map
     * @param none
     * @return none
     */
    Map() {}
    /**
     * @brief Function displayMap
     * @param none
     * @return none
     * The following function displays the map
     * from input .csv file.
     */
    void displayMap();
    /**
     * @brief Function storeMap
     * @param map of type std::vector<int>
     * @return none
     * The following function store the input vector as
     * as the current map.
     */
    void storeMap(std::vector<int> map);
    /**
     * @brief Function loadMap
     * @param mapPath of type string
     * @return none
     * The following and creates a vector of the map
     * from input .csv file.
     */
    void loadMap(std::string mapPath);
    /**
     * @brief Function getMap
     * @param none
     * @return currentMap of type vector<int>
     * The following function returns current map
     * as a vector<int>.
     */
    std::vector<int> getMap();
    /**
     * @brief Function deleteMap
     * @param none
     * @return none
     * The following function to delete the map.
     */
    void deleteMap();
    /**
     * @brief Function returnColumn
     * @param none
     * @return column of type int
     * The following function returns the columns in the map.
     */
    int returnColumn();
    /**
     * @brief Function returnRow
     * @param none
     * @return row of type int
     * The following function returns the rows in the map.
     */
    int returnRow();
    /**
     * @brief Function setColumn
     * @param colCount of type int
     * @return none
     * sets a value for the columns of nodes in the map.
     */
    void setColumn(int colCount);
    /**
     * @brief Function setRow
     * @param rowCount of type int
     * @return none
     * sets a value for the rows of nodes in the map.
     */
    void setRow(int rowCount);
    /**
     * @brief Function returnDirection
     * @param none
     * @return moveDirections of type int array
     * The return the possible movement directions of the robot.
     * (4 in this case north, east, south and west).
     */
    int* returnDirection();
    /**
     * @brief destructor Map
     * @param none
     * @return none
     */
    ~Map() {}
};


void Map::displayMap() {
    std::vector<int> displayLayout = currentMap;
    int node = 1;
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column; j++) {
            if (displayLayout[i*column + j] == 1) {
                std::cout << node++ <<"\t";
            } else {
                std::cout << "x" << "\t";
            }
        }
        std::cout << std::endl;
    }
}

void Map::storeMap(std::vector<int> map) {
    currentMap = map;
}

void Map::loadMap(std::string mapPath) {
    std::ifstream file(mapPath);
    std::string row, cell;
    int rowCount = 0;
    if (file.good()) {
        while (std::getline(file, row)) {
            int columnCount = 0;
            std::istringstream linestream(row);
            while (getline(linestream, cell, ',')) {
                ++columnCount;
                if (cell == "0")
                    currentMap.emplace_back(0);
                else
                    currentMap.emplace_back(1);
            }
            ++rowCount;
            setColumn(columnCount);
        }
    } else {
        std::cout << "The File path entered is not correct";
        setColumn(0);
        setRow(0);
    }
    setRow(rowCount);
}

std::vector<int> Map::getMap() {
    return currentMap;
}

int Map::returnColumn() {
    return column;
}

int Map::returnRow() {
    return row;
}

void Map::setColumn(int colCount) {
    column = colCount;
}

void Map::setRow(int rowCount) {
    row = rowCount;
}

int* Map::returnDirection() {
    return moveDirection;
}
