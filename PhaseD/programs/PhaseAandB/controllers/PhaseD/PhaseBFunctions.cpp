//
//  PhaseAFunctions.cpp
//  
//
//  Created by James Davies on 15/8/21.
//
#include "PhaseBFunctions.hpp"

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define PI 3.14193
#define MOTION_PLAN_FILE_NAME "../../PathPlan.txt"
#define MAP_FILE_NAME "../../../../MapBuilt.txt"
#define CELL_LENGTH_RAD 8.25
//#define CELL_ROTATION 2.2227
#define CELL_ROTATION 2.232

#define TOL1 0.4
#define TOL2 0.009

struct directionCounter {
    int north;
    int south;
    int west;
    int east;
};
struct pathFindingData {
    int robotRow = 0;
    int robotCol = 0;
    int targetRow = 0;
    int targetCol = 0;
    int rowNumber = 11;
    int colNumber = 37;
    char mapMatrix[11][37] = {};
    int horizontalWalls[6][9] = {};
    int verticalWalls[5][10] = {};
    int cellValues[5][9] = {};
    std::vector< std::vector<int> > pathRow;
    std::vector< std::vector<int> > pathCol;
    std::vector<int> currentPathIndex;
    std::vector< std::vector<int> > currentRowCol;
    int pathValues1[5][9] = {};
    int pathValues2[5][9] = {};
    int pathLogical1[5][9] = {};
    int pathLogical2[5][9] = {};
    int directionsChecked[1][4] = {};
    int pathRowColIndeces[1][2] = {};
    int pathIndex;
    int rowPathIndex;
    int colPathIndex;
    int initialRobotRow = 0;
    int initialRobotCol = 0;
    int nextRobotRow = 0;
    int nextRobotCol = 0;
    int pathStepIndex;
    int pathNumber = 0;
    bool finished = 0;
    std::vector<int> turnCounter;
    std::vector< std::vector<char> > possiblePlans;
    std::vector<int> pathStepCount;
    char currentHeading;
    char nextHeading;
    int largestCellValue;
    char initialRobotHeading;
    int motionAction;
};

int determineCol(int colSpaces){
    int col = 0;
    if(colSpaces >= 1 && colSpaces <= 3){
        col = 0;
    }
    if(colSpaces >= 5 && colSpaces <= 7){
        col = 1;
    }
    if(colSpaces >= 9 && colSpaces <= 11){
        col = 2;
    }
    if(colSpaces >= 13 && colSpaces <= 15){
        col = 3;
    }
    if(colSpaces >= 17 && colSpaces <= 19){
        col = 4;
    }
    if(colSpaces >= 21 && colSpaces <= 23){
        col = 5;
    }
    if(colSpaces >= 25 && colSpaces <= 27){
        col = 6;
    }
    if(colSpaces >= 29 && colSpaces <= 31){
        col = 7;
    }
    if(colSpaces >= 33 && colSpaces <= 35){
        col = 8;
    }
    return col;
}

int determineSpace(int colSpaces){
    int col = 0;
    if(colSpaces == 0){
        col = 0;
    }
    if(colSpaces == 4){
        col = 1;
    }
    if(colSpaces == 8){
        col = 2;
    }
    if(colSpaces == 12){
        col = 3;
    }
    if(colSpaces == 16){
        col = 4;
    }
    if(colSpaces == 20){
        col = 5;
    }
    if(colSpaces == 24){
        col = 6;
    }
    if(colSpaces == 28){
        col = 7;
    }
    if(colSpaces == 32){
        col = 8;
    }
    if(colSpaces == 36){
        col = 9;
    }
    return col;
}

void fillHorizontalWalls(int horizontalWalls[6][9], char mapMatrix[11][37], int rowNumber, int colNumber, struct pathFindingData *pf){
    int row = 0;
    int col = 0;
    for (int rowIndex = 0; rowIndex < rowNumber; rowIndex++){
        if(rowIndex % 2 == 0){
            for (int colIndex = 3; colIndex < colNumber; colIndex = colIndex + 4){
                if(mapMatrix[rowIndex][colIndex] == '-'){
                    col = determineCol(colIndex);
                    horizontalWalls[row][col] = 1;
                    pf->horizontalWalls[row][col] = 1;
                } else {
                    col = determineCol(colIndex);
                    horizontalWalls[row][col] = 0;
                    pf->horizontalWalls[row][col] = 0;
                }
            }
            row++;
        }
    }
}

void fillVerticalWalls(int verticalWalls[5][10], char mapMatrix[11][37], int rowNumber, int colNumber, struct pathFindingData *pf){
    int row = 0;
    int col = 0;
    for (int rowIndex = 0; rowIndex < rowNumber; rowIndex++){
        if(rowIndex % 2 != 0){
            for (int colIndex = 0; colIndex < colNumber; colIndex = colIndex + 4){
                if(mapMatrix[rowIndex][colIndex] == '|'){
                    col = determineSpace(colIndex);
                    verticalWalls[row][col] = 1;
                    pf->verticalWalls[row][col] = 1;
                } else {
                    col = determineSpace(colIndex);
                    verticalWalls[row][col] = 0;
                    pf->verticalWalls[row][col] = 0;
                }
            }
            row++;
        }
    }
}

bool checkIfPathRepeated(int pathIndex, int rowPathIndex, int colPathIndex, int robotNextRow, int robotNextCol, std::vector< std::vector<int> > pathRow, std::vector< std::vector<int> > pathCol){
    bool repeated = 0;
    int repeatedSteps[rowPathIndex];
    int repeatedNumber = 0;
    for (int pathToCheck = 0; pathToCheck < pathRow.size()-1; pathToCheck++){
        if (pathToCheck != pathIndex){
            for (int stepIndex = 1; stepIndex < rowPathIndex+1; stepIndex++){
                if (pathRow[pathToCheck][stepIndex] == robotNextRow && pathCol[pathToCheck][stepIndex] == robotNextCol){
                    repeatedSteps[stepIndex] = 1;
                    repeatedNumber++;
                    if (repeatedNumber == rowPathIndex){
                        repeated = 1;
                    }
                }
            }
        }
    }
    return repeated;
}

void printPath(struct pathFindingData *pf, int pathNumber, std::ofstream& output){
    for (int rowIndex = 0; rowIndex < 5; rowIndex++){
        for (int colIndex = 0; colIndex < 9; colIndex++){
            pf->pathLogical1[rowIndex][colIndex] = 50;
        }
    }
    for (int i = 0; i < pf->pathRow[pathNumber].size(); i++){
        pf->pathLogical1[pf->pathRow[pathNumber][i]][pf->pathCol[pathNumber][i]] = 1;
    }
    for (int rowIndex = 0; rowIndex < 5; rowIndex++){
        for (int colIndex = 0; colIndex < 9; colIndex++){
            pf->pathValues1[rowIndex][colIndex] = pf->pathLogical1[rowIndex][colIndex] * pf->cellValues[rowIndex][colIndex];
        }
    }
    int rowCellValues = 0;
    int colCellValues = 0;
    int rowIndex = 0;
    while (rowIndex < 11){
        int colIndex = 0;
        std::cout << "[z5162711_MTRN4110_PhaseB] ";
        output << "[z5162711_MTRN4110_PhaseB] ";
        while (colIndex < 37){
            if (colIndex == 2 || colIndex == 6 || colIndex == 10 || colIndex == 14 || colIndex == 18
                || colIndex == 22 || colIndex == 26 || colIndex == 30 || colIndex == 34) {
                colCellValues = determineCol(colIndex);
                // Print cell values for path
                if (pf->cellValues[rowCellValues][colCellValues] < 10 && rowIndex % 2 != 0) {
                    if(rowCellValues == pf->initialRobotRow && colCellValues == pf->initialRobotCol){
                        std::cout << pf->initialRobotHeading;
                        output << pf->initialRobotHeading;
                        colIndex++;
                    } else if(pf->pathValues1[rowCellValues][colCellValues] < 50){
                        std::cout << pf->pathValues1[rowCellValues][colCellValues];
                        output << pf->pathValues1[rowCellValues][colCellValues];
                        colIndex++;
                    }
                } else if (pf->cellValues[rowCellValues][colCellValues] > 9 && rowIndex % 2 != 0) {
                    if(rowCellValues == pf->initialRobotRow && colCellValues == pf->initialRobotCol){
                        std::cout << pf->initialRobotHeading;
                        output << pf->initialRobotHeading;
                        colIndex++;
                    } else if(pf->pathValues1[rowCellValues][colCellValues] < 50){
                        std::cout << pf->pathValues1[rowCellValues][colCellValues];
                        output << pf->pathValues1[rowCellValues][colCellValues];
                        colIndex = colIndex + 2;
                    }
                }
            }
            // Print map elements
            std::cout << pf->mapMatrix[rowIndex][colIndex];
            output << pf->mapMatrix[rowIndex][colIndex];
            colIndex++;
        }
        if (pf->mapMatrix[rowIndex][0] == '|'){
            rowCellValues++;
        }
        rowIndex++;
        std::cout << std::endl;
        output << "\n";
    }
}


void lookAround(struct pathFindingData *pf, struct directionCounter *dc){
    
    // Pseudocode
    
    // if a direction of movement is valid north
    //      copy the contents of the current path to another array
    //      add the valid cell to the current path
    // then go back to the last cell in the copied array and look around
    // if there is a valid movement east
    //      copy the contents of the current path to another array
    //      add the valid cell to the current path
    // then go back to the last cell in the copied array and look around
    // if there is a valid movement south
    //      copy the contents of the current path to another array
    //      add the valid cell to the current path
    // then go back to the last cell in the copied array and look around
    // if there is a valid movement west
    //      copy the contents of the current path to another array
    //      add the valid cell to the current path
    
    // Go back to the first path's next valid move and repeat the searching and copying process
    // Go back to the second path's next valid move and repeat the searching and copying process
    // Repeat this for all generated paths
    // Repeat this entire process for each subsequent move radiating out from the origin one step at a time for all paths
    // Until you reach the target for each path
    
    
    int end = (int)pf->pathRow.size();
    // check number of paths and for the last cell of each path, set that to be the current position of the robot
    for (int i = 0; i < end; i++) {
        pf->pathIndex = i;
        // Determine the last cell of the path
        pf->robotRow = pf->pathRow[i].back();
        pf->robotCol = pf->pathCol[i].back();
        
        if((pf->robotRow != pf->targetRow) || (pf->robotCol != pf->targetCol)){
            
            int currentPosition = pf->cellValues[pf->robotRow][pf->robotCol];
            
            dc->north++;
            lookNorth(pf, currentPosition, dc);
            dc->south++;
            lookSouth(pf, currentPosition, dc);
            dc->east++;
            lookEast(pf, currentPosition, dc);
            dc->west++;
            lookWest(pf, currentPosition, dc);
            
            if (dc->north == 1 && dc->east == 1 && dc->south == 1 && dc->west == 1){
                dc->north = 0;
                dc->south = 0;
                dc->east = 0;
                dc->west = 0;
                pf->pathRow.erase(pf->pathRow.begin()+((int)pf->pathRow.size()-1));
                pf->pathCol.erase(pf->pathCol.begin()+((int)pf->pathCol.size()-1));
                pf->pathNumber--;
            }
        }
    }
    if((pf->robotRow != pf->targetRow) || (pf->robotCol != pf->targetCol)){
        pf->pathStepIndex++;
    }
}


void lookNorth(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc){
    
    // North
    // if NeighbouringWall does not exist
    if (pf->horizontalWalls[pf->robotRow][pf->robotCol] == 0){
        // if NeighbouringCellValue == N
        if (pf->cellValues[pf->robotRow-1][pf->robotCol] == currentPosition - 1 && pf->robotRow-1 >= 0) {
            // Add a row for the new path branch
            pf->pathRow.resize(pf->pathRow.size()+1);
            pf->pathCol.resize(pf->pathCol.size()+1);
            
            // Copy the last path to a new path
            pf->pathRow[pf->pathNumber+1].assign(pf->pathRow[pf->pathIndex].begin(), pf->pathRow[pf->pathIndex].end());
            pf->pathCol[pf->pathNumber+1].assign(pf->pathCol[pf->pathIndex].begin(), pf->pathCol[pf->pathIndex].end());
            
            // Add new valid cell to last path
            pf->pathRow[pf->pathIndex].resize(pf->pathRow[pf->pathIndex].size()+1);
            pf->pathCol[pf->pathIndex].resize(pf->pathCol[pf->pathIndex].size()+1);
            pf->nextRobotRow = pf->robotRow-1;
            pf->nextRobotCol = pf->robotCol;
            pf->pathRow[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotRow;
            pf->pathCol[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotCol;
            
            pf->pathNumber++;
            pf->pathIndex = pf->pathNumber;
            
        }
    }
}

void lookSouth(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc){
    
    // South
    // if NeighbouringWall does not exist
    if (pf->horizontalWalls[pf->robotRow+1][pf->robotCol] == 0){
        // if NeighbouringCellValue == N
        if (pf->cellValues[pf->robotRow+1][pf->robotCol] == currentPosition - 1){
            
            pf->pathRow.resize(pf->pathRow.size()+1);
            pf->pathCol.resize(pf->pathCol.size()+1);
            
            pf->pathRow[pf->pathNumber+1].assign(pf->pathRow[pf->pathIndex].begin(), pf->pathRow[pf->pathIndex].end());
            pf->pathCol[pf->pathNumber+1].assign(pf->pathCol[pf->pathIndex].begin(), pf->pathCol[pf->pathIndex].end());
            
            pf->pathRow[pf->pathIndex].resize(pf->pathRow[pf->pathIndex].size()+1);
            pf->pathCol[pf->pathIndex].resize(pf->pathCol[pf->pathIndex].size()+1);
            
            pf->nextRobotRow = pf->robotRow+1;
            pf->nextRobotCol = pf->robotCol;
            pf->pathRow[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotRow;
            pf->pathCol[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotCol;
            
            pf->pathNumber++;
            pf->pathIndex = pf->pathNumber;
            
        }
    }
}

void lookEast(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc){
    
    //East
    // if NeighbouringWall does not exist
    if (pf->verticalWalls[pf->robotRow][pf->robotCol+1] == 0){
        // if NeighbouringCellValue == N
        if (pf->cellValues[pf->robotRow][pf->robotCol+1] == currentPosition - 1){
            
            pf->pathRow.resize(pf->pathRow.size()+1);
            pf->pathCol.resize(pf->pathCol.size()+1);
            
            pf->pathRow[pf->pathNumber+1].assign(pf->pathRow[pf->pathIndex].begin(), pf->pathRow[pf->pathIndex].end());
            pf->pathCol[pf->pathNumber+1].assign(pf->pathCol[pf->pathIndex].begin(), pf->pathCol[pf->pathIndex].end());
            
            pf->pathRow[pf->pathIndex].resize(pf->pathRow[pf->pathIndex].size()+1);
            pf->pathCol[pf->pathIndex].resize(pf->pathCol[pf->pathIndex].size()+1);
            
            pf->nextRobotRow = pf->robotRow;
            pf->nextRobotCol = pf->robotCol+1;
            pf->pathRow[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotRow;
            pf->pathCol[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotCol;
            
            pf->pathNumber++;
            pf->pathIndex = pf->pathNumber;
            
        }
    }
}

void lookWest(struct pathFindingData *pf, int currentPosition, struct directionCounter *dc){
    
    // West
    // if NeighbouringWall does not exist
    if (pf->verticalWalls[pf->robotRow][pf->robotCol] == 0){
        // if NeighbouringCellValue == N
        if (pf->cellValues[pf->robotRow][pf->robotCol-1] == currentPosition - 1 && pf->robotCol-1 >= 0){
            
            pf->pathRow.resize(pf->pathRow.size()+1);
            pf->pathCol.resize(pf->pathCol.size()+1);
            
            pf->pathRow[pf->pathNumber+1].assign(pf->pathRow[pf->pathIndex].begin(), pf->pathRow[pf->pathIndex].end());
            pf->pathCol[pf->pathNumber+1].assign(pf->pathCol[pf->pathIndex].begin(), pf->pathCol[pf->pathIndex].end());
            
            pf->pathRow[pf->pathIndex].resize(pf->pathRow[pf->pathIndex].size()+1);
            pf->pathCol[pf->pathIndex].resize(pf->pathCol[pf->pathIndex].size()+1);
            
            pf->nextRobotRow = pf->robotRow;
            pf->nextRobotCol = pf->robotCol-1;
            pf->pathRow[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotRow;
            pf->pathCol[pf->pathIndex][pf->pathStepIndex+1] = pf->nextRobotCol;
            
            pf->pathNumber++;
            pf->pathIndex = pf->pathNumber;
        }
    }
}

