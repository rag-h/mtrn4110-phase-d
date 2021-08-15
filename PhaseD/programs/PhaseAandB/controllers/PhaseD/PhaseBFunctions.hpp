//
//  PhaseBFunctions.hpp
//  
//
//  Created by James Davies on 15/8/21.
//

#ifndef PhaseBFunctions_hpp
#define PhaseBFunctions_hpp

#include <iostream>
#include <fstream>
#include <string>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <sstream>

// Function Definitions - Phase B
int determineCol(int colSpaces);
int determineSpace(int colSpaces);
void fillHorizontalWalls(int horizontalWalls[6][9], char mapMatrix[11][37], int rowNumber, int colNumber, struct pathFindingData *pf);
void fillVerticalWalls(int verticalWalls[5][10], char mapMatrix[11][37], int rowNumber, int colNumber, struct pathFindingData *pf);
bool checkIfPathRepeated(int pathIndex, int rowPathIndex, int colPathIndex, int robotNextRow, int robotNextCol, std::vector< std::vector<int> > pathRow, std::vector< std::vector<int> > pathCol);
void printPath(struct pathFindingData *pf, int pathNumber, std::ofstream& ouput);
void lookAround(struct pathFindingData *pf, struct directionCounter *dc);
void lookNorth(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc);
void lookSouth(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc);
void lookEast(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc);
void lookWest(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc);


#endif /* PhaseBFunctions_hpp */
