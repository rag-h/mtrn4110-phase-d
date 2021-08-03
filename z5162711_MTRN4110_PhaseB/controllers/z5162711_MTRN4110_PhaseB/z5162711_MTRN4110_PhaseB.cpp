// File:          z5162711_MTRN4110_PhaseB.cpp
// Date:          23/06/2021
// Description:   Controller of E-Puck for Phase B - Path Planning
// Author:        James Jonathon Davies
// Modifications:
// Platform:      MacOS

#include <iostream>
#include <fstream>
#include <string>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <limits>
#include <iostream>
#include <vector>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define PI 3.14193
#define MAP_FILE_NAME "../../Map.txt"
#define CELL_LENGTH_RAD 8.25
#define CELL_ROTATION 2.2227

// Function Definitions
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

int main(int argc, char **argv) {
    
    const char *OUTPUT_FILE_NAME = "../../Output.txt";
    remove(OUTPUT_FILE_NAME);
    std::ofstream output(OUTPUT_FILE_NAME, std::fstream::app );
    
    // Read motion plan
    std::cout << "[z5162711_MTRN4110_PhaseB] Reading in map from ../../Map.txt..." << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] Reading in map from ../../Map.txt...\n";
    
    std::ifstream file;               // create file stream
    file.open(MAP_FILE_NAME);         // open file and connect to file stream
    std::string map;                  // initialise string for motion plan
    std::string fileContents;
    // std::string map;
    // std::getline(file, map);          // get motion plan string from stream
    
    // Initialise Variables
    struct pathFindingData pf;
    struct directionCounter dc;
    
    int row = 0;
    int col = 0;
    int colSpaces = 0;
    int rowSpaces = 0;
    char heading = 'S';
    int robotRow = 0;
    int robotCol = 0;
    int targetRow = 0;
    int targetCol = 0;
    int rowNumber = 11;
    int colNumber = 37;
    char mapMatrix[11][37] = {};
    int horizontalWalls[6][9];
    int verticalWalls[5][10];
    int cellValues[5][9];
    
    for (int lineNumber = 0; std::getline(file,map) && lineNumber < rowNumber; lineNumber++){
        
        std::cout << "[z5162711_MTRN4110_PhaseB] " << map << std::endl;
        output << "[z5162711_MTRN4110_PhaseB] " << map << "\n";
        while(colSpaces < colNumber){
            
            col = determineCol(colSpaces);
            
            if(map[colSpaces] == '^'){
                heading = 'N';
                pf.currentHeading = heading;
                pf.initialRobotHeading ='^';
                robotCol = determineCol(colSpaces);
                robotRow = row;
                pf.initialRobotRow = robotRow;
                pf.initialRobotCol = robotCol;
            }
            if(map[colSpaces] == '>'){
                heading = 'E';
                pf.currentHeading = heading;
                pf.initialRobotHeading ='>';
                robotCol = determineCol(colSpaces);
                robotRow = row;
                pf.initialRobotRow = robotRow;
                pf.initialRobotCol = robotCol;
            }
            if(map[colSpaces] == 'v'){
                heading = 'S';
                pf.currentHeading = heading;
                pf.initialRobotHeading ='v';
                robotCol = determineCol(colSpaces);
                robotRow = row;
                pf.initialRobotRow = robotRow;
                pf.initialRobotCol = robotCol;
            }
            if(map[colSpaces] == '<'){
                heading = 'W';
                pf.currentHeading = heading;
                pf.initialRobotHeading ='<';
                robotCol = determineCol(colSpaces);
                robotRow = row;
                pf.initialRobotRow = robotRow;
                pf.initialRobotCol = robotCol;
            }
            if(map[colSpaces] == 'x'){
                targetCol = determineCol(colSpaces);
                targetRow = row;
            }
            
            // std::cout << "[z5162711_MTRN4110_PhaseB] Row/Col: " << map[colSpaces] << std::endl;
            // std::cout << "[z5162711_MTRN4110_PhaseB] Row/Col: " << rowSpaces << ',' << colSpaces << std::endl;
            mapMatrix[rowSpaces][colSpaces] = map[colSpaces];
            pf.mapMatrix[rowSpaces][colSpaces] = map[colSpaces];
            colSpaces++;
            
        }
        colSpaces = 0;
        if (map[0] == '|'){
            row++;
        }
        rowSpaces++;
    }
    
    fillHorizontalWalls(horizontalWalls, mapMatrix, rowNumber, colNumber, &pf);
    fillVerticalWalls(verticalWalls, mapMatrix, rowNumber, colNumber, &pf);
    
    //    for (int rowIndex = 0; rowIndex < 6; rowIndex++){
    //        std::cout << "[z5162711_MTRN4110_PhaseB] Horizontal Walls: " << horizontalWalls[rowIndex][0]
    //        << horizontalWalls[rowIndex][1]
    //        << horizontalWalls[rowIndex][2]
    //        << horizontalWalls[rowIndex][3]
    //        << horizontalWalls[rowIndex][4]
    //        << horizontalWalls[rowIndex][5]
    //        << horizontalWalls[rowIndex][6]
    //        << horizontalWalls[rowIndex][7]
    //        << horizontalWalls[rowIndex][8]
    //        << std::endl;
    //    }
    //
    //    for (int rowIndex = 0; rowIndex < 5; rowIndex++){
    //        std::cout << "[z5162711_MTRN4110_PhaseB] Vertical   Walls: " << verticalWalls[rowIndex][0]
    //        << verticalWalls[rowIndex][1]
    //        << verticalWalls[rowIndex][2]
    //        << verticalWalls[rowIndex][3]
    //        << verticalWalls[rowIndex][4]
    //        << verticalWalls[rowIndex][5]
    //        << verticalWalls[rowIndex][6]
    //        << verticalWalls[rowIndex][7]
    //        << verticalWalls[rowIndex][8]
    //        << verticalWalls[rowIndex][9]
    //        << std::endl;
    //    }
    
    
    std::cout << "[z5162711_MTRN4110_PhaseB] Map read in!" << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] Map read in!\n";
    //  FLOOD FILL ALGORITHM PSEUDOCODE
    //  initialize
    //    all CellValues <- N (N is a big number, e.g. N = Rows x Columns)
    //    TargetCellValue <- 0
    //    CurrentExploredValue <- 0
    //    MazeValueChanged <- 1
    //  while MazeValueChanged ≠ 0
    //    MazeValueChanged <- 0
    //    forall Rows
    //      forall Columns
    //        if CurrentCellValue == CurrentExploredValue
    //          forall Directions(North, East, South, West)
    //            if NeighbouringWall does not exist
    //              if NeighbouringCellValue == N
    //                NeighbouringCellValue <- CurrentCellValue + 1
    //                MazeValueChanged <- 1
    //    CurrentExploredValue = CurrentExploredValue + 1
    //  return
    
    
    std::cout << "[z5162711_MTRN4110_PhaseB] Finding shortest paths..." << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] Finding shortest paths...\n";
    // Initialise cell values
    // all CellValues <- N (N is a big number, e.g. N = Rows x Columns)
    int N = 5 * 9;
    for (int rowIndex = 0; rowIndex < 5; rowIndex++){
        for (int colIndex = 0; colIndex < 9; colIndex++){
            cellValues[rowIndex][colIndex] = N;
            pf.cellValues[rowIndex][colIndex] = N;
        }
    }
    // TargetCellValue <- 0
    cellValues[targetRow][targetCol] = 0;
    pf.cellValues[targetRow][targetCol] = 0;
    pf.largestCellValue = 0;
    // CurrentExploredValue <- 0
    int currentlyExploredValue = 0;
    // MazeValueChanged <- 1
    int mazeValueChanged = 1;
    // while MazeValueChanged ≠ 0
    while (mazeValueChanged != 0 && currentlyExploredValue < N){
        // MazeValueChanged <- 0
        int mazeValueChanged = 0;
        // forall Rows
        for (int rowIndex = 0; rowIndex < 5; rowIndex++){
            // forall Columns
            for (int colIndex = 0; colIndex < 9; colIndex++){
                // if CurrentCellValue == CurrentExploredValue
                if (cellValues[rowIndex][colIndex] == currentlyExploredValue){
                    // forall Directions(North, East, South, West)
                    // North
                    // if NeighbouringWall does not exist
                    if (horizontalWalls[rowIndex][colIndex] == 0){
                        // if NeighbouringCellValue == N
                        if (cellValues[rowIndex-1][colIndex] == N && rowIndex-1 >= 0) {
                            // NeighbouringCellValue <- CurrentCellValue + 1
                            cellValues[rowIndex-1][colIndex] = cellValues[rowIndex][colIndex] + 1;
                            // Check if this value is the largest so far, if so, save it
                            if(cellValues[rowIndex][colIndex] + 1 > pf.largestCellValue){
                                pf.largestCellValue = cellValues[rowIndex][colIndex] + 1;
                            }
                            pf.cellValues[rowIndex-1][colIndex] = pf.cellValues[rowIndex][colIndex] + 1;
                            // MazeValueChanged <- 1
                            mazeValueChanged = 1;
                        }
                    }
                    // South
                    // if NeighbouringWall does not exist
                    if (horizontalWalls[rowIndex+1][colIndex] == 0){
                        // if NeighbouringCellValue == N
                        if (cellValues[rowIndex+1][colIndex] == N){
                            // NeighbouringCellValue <- CurrentCellValue + 1
                            cellValues[rowIndex+1][colIndex] = cellValues[rowIndex][colIndex] + 1;
                            // Check if this value is the largest so far, if so, save it
                            if(cellValues[rowIndex][colIndex] + 1 > pf.largestCellValue){
                                pf.largestCellValue = cellValues[rowIndex][colIndex] + 1;
                            }
                            pf.cellValues[rowIndex+1][colIndex] = pf.cellValues[rowIndex][colIndex] + 1;
                            // MazeValueChanged <- 1
                            mazeValueChanged = 1;
                        }
                    }
                    // East
                    // if NeighbouringWall does not exist
                    if (verticalWalls[rowIndex][colIndex+1] == 0){
                        // if NeighbouringCellValue == N
                        if (cellValues[rowIndex][colIndex+1] == N){
                            // NeighbouringCellValue <- CurrentCellValue + 1
                            cellValues[rowIndex][colIndex+1] = cellValues[rowIndex][colIndex] + 1;
                            // Check if this value is the largest so far, if so, save it
                            if(cellValues[rowIndex][colIndex] + 1 > pf.largestCellValue){
                                pf.largestCellValue = cellValues[rowIndex][colIndex] + 1;
                            }
                            pf.cellValues[rowIndex][colIndex+1] = pf.cellValues[rowIndex][colIndex] + 1;
                            // MazeValueChanged <- 1
                            mazeValueChanged = 1;
                        }
                    }
                    // West
                    // if NeighbouringWall does not exist
                    if (verticalWalls[rowIndex][colIndex] == 0){
                        // if NeighbouringCellValue == N
                        if (cellValues[rowIndex][colIndex-1] == N && colIndex-1 >= 0){
                            // NeighbouringCellValue <- CurrentCellValue + 1
                            cellValues[rowIndex][colIndex-1] = cellValues[rowIndex][colIndex] + 1;
                            // Check if this value is the largest so far, if so, save it
                            if(cellValues[rowIndex][colIndex] + 1 > pf.largestCellValue){
                                pf.largestCellValue = cellValues[rowIndex][colIndex] + 1;
                            }
                            pf.cellValues[rowIndex][colIndex-1] = pf.cellValues[rowIndex][colIndex] + 1;
                            // MazeValueChanged <- 1
                            mazeValueChanged = 1;
                        }
                    }
                }
            }
        }
        // CurrentExploredValue = CurrentExploredValue + 1
        currentlyExploredValue++;
        
    }
    
    //    for (int rowIndex = 0; rowIndex < 5; rowIndex++){
    //        std::cout << "[z5162711_MTRN4110_PhaseB] Vertical   Walls: " << cellValues[rowIndex][0] << " "
    //                                                                    << cellValues[rowIndex][1] << " "
    //                                                                    << cellValues[rowIndex][2] << " "
    //                                                                    << cellValues[rowIndex][3] << " "
    //                                                                    << cellValues[rowIndex][4] << " "
    //                                                                    << cellValues[rowIndex][5] << " "
    //                                                                    << cellValues[rowIndex][6] << " "
    //                                                                    << cellValues[rowIndex][7] << " "
    //                                                                    << cellValues[rowIndex][8]
    //                                                                    << std::endl;
    //    }
    
    // Print out flood filled map
    //    int rowCellValues = 0;
    //    int colCellValues = 0;
    //    int rowIndex = 0;
    //    while (rowIndex < 11){
    //        int colIndex = 0;
    //        std::cout << "[z5162711_MTRN4110_PhaseB] ";
    //        while (colIndex < 37){
    //            if (colIndex == 2 || colIndex == 6 || colIndex == 10 || colIndex == 14 || colIndex == 18
    //                || colIndex == 22 || colIndex == 26 || colIndex == 30 || colIndex == 34) {
    //                colCellValues = determineCol(colIndex);
    //                if (cellValues[rowCellValues][colCellValues] < 10 && rowIndex % 2 != 0) {
    //                    std::cout << cellValues[rowCellValues][colCellValues];
    //                    colIndex++;
    //                }
    //                if (cellValues[rowCellValues][colCellValues] > 9 && rowIndex % 2 != 0) {
    //                    std::cout << cellValues[rowCellValues][colCellValues];
    //                    colIndex = colIndex + 2;
    //                }
    //            }
    //            std::cout << mapMatrix[rowIndex][colIndex];
    //
    //            colIndex++;
    //        }
    //
    //        if (mapMatrix[rowIndex][0] == '|'){
    //            rowCellValues++;
    //        }
    //
    //        rowIndex++;
    //        std::cout << std::endl;
    //    }
    
    
    //    std::cout << "[z5162711_MTRN4110_PhaseB] Robot Pose: " <<  pf.robotRow << pf.robotCol << heading << std::endl;
    //    std::cout << "[z5162711_MTRN4110_PhaseB] Target: " <<  pf.targetRow << "," << pf.targetCol << std::endl;
    
    dc.north = 0;
    dc.south = 0;
    dc.east = 0;
    dc.west = 0;
    pf.robotRow = robotRow;
    pf.robotCol = robotCol;
    pf.targetRow = targetRow;
    pf.targetCol = targetCol;
    pf.pathRow.resize(1);
    pf.pathCol.resize(1);
    pf.pathRow[0].resize(1);
    pf.pathCol[0].resize(1);
    pf.pathRow[0][0] = robotRow;
    pf.pathCol[0][0] = robotCol;
    pf.pathIndex = 0;
    pf.rowPathIndex = 1;
    pf.colPathIndex = 1;
    pf.nextRobotRow = 0;
    pf.nextRobotCol = 0;
    pf.currentPathIndex.resize(1);
    pf.currentPathIndex[0] = 0;
    pf.currentRowCol.resize(1);
    pf.currentRowCol[0].resize(2);
    pf.currentRowCol[0][0] = robotRow;
    pf.currentRowCol[0][1] = robotCol;
    pf.pathStepIndex = 0;
    pf.pathNumber = 0;
    pf.possiblePlans.resize(1);
    pf.possiblePlans[0].resize(1);
    
    // Search for shortest paths
    int stopSearch = 0;
    while (!stopSearch){
        
        lookAround(&pf, &dc);
        
        int end = (int)pf.pathRow.size();
        for (int i = 0; i < end; i++) {
            if(pf.pathRow[i].back() != pf.targetRow || pf.pathCol[i].back() != pf.targetCol){
                pf.finished = 0;
            } else {
                pf.finished = 1;
            }
        }
        if(pf.finished == 1){
            stopSearch = 1;
        }
        
    }
    
    // Print out shortest paths
    int counter = 0;
    int end = (int)pf.pathRow.size();
    for (int i = 0; i < end; i++) {
        std::cout << "[z5162711_MTRN4110_PhaseB] Path - " << i+1 << ":" << std::endl;
        output << "[z5162711_MTRN4110_PhaseB] Path -  " << i+1 << "\n";
        printPath(&pf,i,output);
        counter++;
    }
    std::cout << "[z5162711_MTRN4110_PhaseB] " << counter << " shortest paths found!" << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] " << counter << " shortest paths found!\n";
    // Find shortest path with the least number of turns
    pf.turnCounter.resize((int)pf.pathRow.size());
    pf.currentHeading = heading;
    pf.possiblePlans.resize((int)pf.pathRow.size());
    
    
    // For each path
    for (int i = 0; i < (int)pf.pathRow.size(); i++) {
        pf.possiblePlans[i].resize(2);
        pf.possiblePlans[i][0] = pf.initialRobotRow + 48;
        pf.possiblePlans[i][1] = pf.initialRobotCol + 48;
        int possiblePlansCounter = 3;
        int firstMove = 1;
        
        if (i > 0){
            pf.possiblePlans[i-1].resize(pf.possiblePlans[i-1].size()-1);
        }
        
        // Check if the current heading is different from the next heading
        // If it is then add a turn to the counter for that path
        for (int j = 0; j < (int)pf.pathRow[i].size(); j++){
            pf.possiblePlans[i].resize(pf.possiblePlans[i].size()+1);
            
            // Determine the current heading
            if (firstMove == 1) {
                pf.possiblePlans[i].resize(pf.possiblePlans[i].size()+1);
                pf.currentHeading = heading;
                pf.possiblePlans[i][2] = heading;
            } else {
                pf.currentHeading = pf.nextHeading;
            }
            firstMove = 0;
            // Determine the next heading
            if (j + 1 < (int)pf.pathRow[i].size()){
                // If no change in row number
                if(pf.pathRow[i][j] - pf.pathRow[i][j+1] == 0){
                    // If next cell is to the WEST
                    if(pf.pathCol[i][j] - pf.pathCol[i][j+1] < 0){
                        pf.nextHeading = 'E';
                    }
                    // If next cell is to the EAST
                    else if(pf.pathCol[i][j] - pf.pathCol[i][j+1] > 0){
                        pf.nextHeading = 'W';
                    }
                }
                // If next cell is to the SOUTH
                else if(pf.pathRow[i][j] - pf.pathRow[i][j+1] < 0){
                    pf.nextHeading = 'S';
                }
                // If next cell is to the NORTH
                else if(pf.pathRow[i][j] - pf.pathRow[i][j+1] > 0){
                    pf.nextHeading = 'N';
                }
                
                // If robot turning then we need to stay at the current cell as the pathRow and pathCol should stay the same
                // so we need to decrement the pathRow/Col index but increment the possiblePlans index
                if (pf.currentHeading != pf.nextHeading){
                    // Right Turn
                    if(pf.currentHeading == 'N' && pf.nextHeading == 'E'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'R';
                    } else if(pf.currentHeading == 'E' && pf.nextHeading == 'S'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'R';
                    } else if(pf.currentHeading == 'S' && pf.nextHeading == 'W'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'R';
                    } else if(pf.currentHeading == 'W' && pf.nextHeading == 'N'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'R';
                    }
                    // Left Turn
                    else if(pf.currentHeading == 'N' && pf.nextHeading == 'W'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                    } else if(pf.currentHeading == 'W' && pf.nextHeading == 'S'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                    } else if(pf.currentHeading == 'S' && pf.nextHeading == 'E'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                    } else if(pf.currentHeading == 'E' && pf.nextHeading == 'N'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                    }
                    
                    else if(pf.currentHeading == 'N' && pf.nextHeading == 'S'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                        pf.nextHeading = 'W';
                    } else if(pf.currentHeading == 'S' && pf.nextHeading == 'N'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                        pf.nextHeading = 'E';
                    } else if(pf.currentHeading == 'W' && pf.nextHeading == 'E'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                        pf.nextHeading = 'S';
                    } else if(pf.currentHeading == 'E' && pf.nextHeading == 'W'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                        pf.nextHeading = 'N';
                    }
                    
                    if (possiblePlansCounter > 3 && pf.possiblePlans[i][possiblePlansCounter] == 'R' && pf.possiblePlans[i][possiblePlansCounter-1] == 'R'){
                        pf.possiblePlans[i][possiblePlansCounter] = 'L';
                        pf.possiblePlans[i][possiblePlansCounter-1] = 'L';
                    }
                    pf.currentHeading = pf.nextHeading;
                    pf.turnCounter[i]++;
                    possiblePlansCounter++;
                    j--;
                } else {
                    // Forward
                    pf.possiblePlans[i][possiblePlansCounter] = 'F';
                    possiblePlansCounter++;
                }
            }
        }
    }
    pf.possiblePlans[pf.possiblePlans.size()-1].resize(pf.possiblePlans[pf.possiblePlans.size()-1].size()-1);
    
    pf.pathStepCount.resize((int)pf.possiblePlans.size());
    for (int i = 0; i < (int)pf.possiblePlans.size(); i++) {
        pf.pathStepCount[i] = ((int)pf.possiblePlans[i].size() - 3);
    }
    
    // Print all paths
    //    for (int i = 0; i < (int)pf.possiblePlans.size(); i++) {
    //        std::cout << "[z5162711_MTRN4110_PhaseB] Path plan (" << pf.pathStepCount[i] << " steps): ";
    //        for (int j = 0; j < (int)pf.possiblePlans[i].size(); j++){
    //
    //            std::cout <<  pf.possiblePlans[i][j];
    //
    //        }
    //        std::cout << std::endl;
    //    }
    
    // Find the index of the path with the fewest turns
    std::cout << "[z5162711_MTRN4110_PhaseB] Finding shortest path with least turns..." << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] Finding shortest path with least turns...\n";
    int fewestTurnsIndex = 0;
    int fewestTurns = *min_element(pf.turnCounter.begin(), pf.turnCounter.end());
    for (int i = 0; i < (int)pf.turnCounter.size(); i++) {
        
        if(pf.turnCounter[i] == fewestTurns){
            fewestTurnsIndex = i;
        }
        
    }
    
    printPath(&pf,fewestTurnsIndex, output);
    
    // Print shortest path
    std::cout << "[z5162711_MTRN4110_PhaseB] Shortest path with least turns found!" << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] Shortest path with least turns found!\n";
    std::cout << "[z5162711_MTRN4110_PhaseB] Path Plan (" << pf.pathStepCount[fewestTurnsIndex] << " steps): ";
    output << "[z5162711_MTRN4110_PhaseB] Path Plan (" << pf.pathStepCount[fewestTurnsIndex] << " steps): ";
    for (int j = 0; j < (int)pf.possiblePlans[fewestTurnsIndex].size(); j++){
        
        std::cout <<  pf.possiblePlans[fewestTurnsIndex][j];
        output << pf.possiblePlans[fewestTurnsIndex][j];
        
    }
    std::cout << std::endl;
    output << "\n";
    
    std::cout << "[z5162711_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt..." << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt...\n";
    const char *PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
    remove(PATH_PLAN_FILE_NAME);
    std::ofstream pathPlan(PATH_PLAN_FILE_NAME, std::fstream::app );
    
    for (int j = 0; j < (int)pf.possiblePlans[fewestTurnsIndex].size(); j++){
        
        pathPlan << pf.possiblePlans[fewestTurnsIndex][j];
        
    }
    
    std::cout << "[z5162711_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!" << std::endl;
    output << "[z5162711_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!\n";
    
    return 0;
}






// FUNCTIONS

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


