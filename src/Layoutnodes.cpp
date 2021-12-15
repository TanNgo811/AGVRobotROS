

#pragma once
#include <iostream>

/**
 * @brief Class Layoutnodes
 * The following class Layoutnodes stores details regarding the nodes
 * of a map. It store data regarding index of the node, its parent,
 * costs, row and column index.
 */
class Layoutnodes {
 private:
    int nodeIndex;
    int parentIndex;
    int rowIndex;
    int columnIndex;
    double heuristicCost;
    double pathCost;
    double totalCost;

 public:
    /**
     * @brief constructor Layoutnodes
     * @param none
     * @return none
     * initializes private variables.
     */
     Layoutnodes() {
         nodeIndex = 999;
         parentIndex = 9999;
         rowIndex = 0;
         columnIndex = 0;
         heuristicCost = 0;
         pathCost = 0;
         totalCost = 0;
     }
    /**
     * @brief Function setNodeIndex
     * @param index of type int
     * @param column of type int
     * @param row of type int
     * @return none
     * The following function stores the index for a node
     */
    void setNodeIndex(int index, int column, int row);
    /**
     * @brief Function setParentIndex
     * @param pIndex of type int
     * @return none
     * The following function initializes the parent (pIndex)
     * of the node.
     */
    void setParentIndex(int pIndex);
    /**
     * @brief Function getParentIndex
     * @param none
     * @return parentIndex of type int
     * Returns the the parent index of the node.
     */
    int getParentIndex();
    /**
     * @brief Function setHeuristicCost
     * @param heuristic of type double
     * @return none
     * Stores the heuristic cost of the node.
     */
    void setHeuristicCost(double heuristic);
    /**
     * @brief Function setPathCost
     * @param cost of type double
     * @return none
     * The following function sets the pathCost
     * of the node.
     */
    void setPathCost(double cost);
    /**
     * @brief Function setTotalCost
     * @param none
     * @return none
     * The following function calculates the totalCost 
     * from heuristic and path cost.
     */
    void setTotalCost();
    /**
     * @brief Function setCost
     * @param cost of type double
     * @return none
     * The following function sets the totalcost
     * of the node.
     */
    void setCost(double cost);
    /**
     * @brief Function output
     * @param none
     * @return none
     * Prints row and colum index (used in testing).
     */
    void output();
    /**
     * @brief Function getCost
     * @param none
     * @return totalCost of type double
     * The following function returns the totalCost of the node
     */
    double getCost();
    /**
     * @brief Function getRowIndex
     * @param none
     * @return rowIndex of type int
     * The following function returns rowIndex of the node.
     */
    int getRowIndex();
    /**
     * @brief Function getColumnIndex
     * @param none
     * @return columnIndex of type int
     * Returns column index of the node.
     */
    int getColumnIndex();
    /**
     * @brief Function getIndex
     * @param none
     * @return nodeIndex of type int
     * The following function returns the node index
     */
    int getIndex();
    /**
     * @brief Function returnHCost
     * @param none
     * @return heursiticCost of type double
     * The following function returns the hueristic cost
     * of the node.
     */
    double returnHCost();
    /**
     * @brief destructor Layoutnodes
     * @param none
     * @return none
     */
    ~Layoutnodes() {}
};

void Layoutnodes::setNodeIndex(int index, int column, int row) {
    nodeIndex = index;
    columnIndex = column;
    rowIndex = row;
    }

void Layoutnodes::setParentIndex(int pIndex) {
    parentIndex = pIndex;
}

int Layoutnodes::getParentIndex() {
    return parentIndex;
}

void Layoutnodes::setHeuristicCost(double heuristic) {
    heuristicCost = heuristic;
}

void Layoutnodes::setPathCost(double cost) {
    pathCost = cost;
}

void Layoutnodes::setTotalCost() {
    totalCost = pathCost + 0.1*heuristicCost;
}

void Layoutnodes::setCost(double cost) {
    totalCost = cost;
}

void Layoutnodes::output() {
    std::cout << rowIndex << " " << columnIndex << std::endl;
}

double Layoutnodes::getCost() {
    return totalCost;
}

int Layoutnodes::getRowIndex() {
    return rowIndex;
}

int Layoutnodes::getColumnIndex() {
    return columnIndex;
}

int Layoutnodes::getIndex() {
    return nodeIndex;
}

double Layoutnodes::returnHCost() {
    return heuristicCost;
}

