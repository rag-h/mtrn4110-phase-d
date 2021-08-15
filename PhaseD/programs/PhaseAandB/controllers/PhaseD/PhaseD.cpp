// File:          z5162711_MTRN4110_phaseA.cpp
// Date:          09/06/2021
// Description:   Controller of E-Puck for Phase A - Driving and Perception
// Author:        James Jonathon Davies
// Modifications:
// Platform:      MacOS

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/RangeFinder.hpp>
#include <iostream>
#include "PhaseAFunctions.cpp"
#include "PhaseBFunctions.cpp"

using namespace webots;


int main(int argc, char **argv) {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Phase C

  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Phase B
  
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
  
  std::string pathString;
  
  for (int j = 0; j < (int)pf.possiblePlans[fewestTurnsIndex].size(); j++){
      
      pathPlan << pf.possiblePlans[fewestTurnsIndex][j];
      
      pathString.push_back(pf.possiblePlans[fewestTurnsIndex][j]);
      
  }
  
  std::cout << "[z5162711_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!" << std::endl;
  output << "[z5162711_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!\n";
  
  std::cout << pathString << std::endl;
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Phase A
  
  std::ofstream myfile;  
  myfile.open (MOTION_PLAN_FILE_NAME);  
  if(file.is_open()){  
    std::cout << "File is open" << std::endl;  
    myfile.close();//file close  
    std::cout << "File close successfully." << std::endl;  
  }  
  else{  
    std::cout<< "Error in file opening" << std::endl;  
  }  
  
  // Read motion plan
  std::cout << "[z5162711_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
  
  std::ifstream f;               // create file stream
  f.open(MOTION_PLAN_FILE_NAME); // open file and connect to file stream
  std::string motionPlan;        // initialise string for motion plan
  std::getline(f, motionPlan);   // get motion plan string from stream
  motionPlan = pathString;
  std::cout << "[z5162711_MTRN4110_PhaseA] Motion Plan: " << motionPlan << std::endl;
  std::cout << "[z5162711_MTRN4110_PhaseA] Motion plan read in!" << std::endl;

  // Initialise robot
  Robot *robot = new Robot();
  
  // Initialise front laser sensor
  DistanceSensor *dsF[1];
  char dsFront[1][4] = {"dsF"};
  dsF[0] = robot->getDistanceSensor(dsFront[0]);
  dsF[0] -> enable(TIME_STEP);
  
  // Initialise right laser sensor
  DistanceSensor *dsR[1];
  char dsRight[1][4] = {"dsR"};
  dsR[0] = robot->getDistanceSensor(dsRight[0]);
  dsR[0] -> enable(TIME_STEP);
    
  // Initialise left laser sensor
  DistanceSensor *dsL[1];
  char dsLeft[1][4] = {"dsL"};
  dsL[0] = robot->getDistanceSensor(dsLeft[0]);
  dsL[0] -> enable(TIME_STEP);
    
  // Initialise wheel encoders
  PositionSensor *rs[2];
  char rsNames[2][19] = {"left wheel sensor", "right wheel sensor"};
  rs[0] = robot->getPositionSensor(rsNames[0]);
  rs[0] -> enable(TIME_STEP);
  rs[1] = robot->getPositionSensor(rsNames[1]);
  rs[1] -> enable(TIME_STEP);
  
  // Initialise compass
  Compass *compass[1];
  char compassTurret[1][8] = {"compass"};
  compass[0] = robot->getCompass(compassTurret[0]);
  compass[0] -> enable(TIME_STEP);
  
  // Initialise range-finder
  RangeFinder *rangeFinder[1];
  char rangeFinderTurret[1][13] = {"range-finder"};
  rangeFinder[0] = robot->getRangeFinder(rangeFinderTurret[0]);
  rangeFinder[0] -> enable(TIME_STEP);
  
  // initialise wheel motors
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(0);
  rightMotor->setPosition(0);
  leftMotor->setVelocity(0*MAX_SPEED);
  rightMotor->setVelocity(0*MAX_SPEED);
  

  std::cout << "[z5162711_MTRN4110_PhaseA] Executing motion plan..." << std::endl;

  // Initialise Variables
  int step = 3;
  char rowChar = motionPlan[0];
  char colChar = motionPlan[1];
  row = (int)rowChar - 48;;
  col = (int)colChar - 48;;
  heading = motionPlan[2];
  char lWall = 'N';
  char rWall = 'N';
  char fWall = 'N';
  char walls [4];
  int dsFValue = 0;
  int dsRValue = 0;
  int dsLValue = 0;
  double currentLW = 0;
  double currentRW = 0;
  double nextLW = 0;
  double nextRW = 0;
  int lastStepComplete = 0;
  int stepNearlyDone = 0;
  int turning = 0;
  int forward = 0;
  double currentRange = 0;
  double compassBearing = 0;
  
  // Initialise csv writing stuff
  const char *MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
  remove(MOTION_EXECUTION_FILE_NAME);
  std::ofstream motionExecution(MOTION_EXECUTION_FILE_NAME, std::fstream::app );
  motionExecution << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall,\n";
 
  // MAIN LOOP    
  while (robot->step(TIME_STEP) != -1 && step < (motionPlan.length()+1)){
    
    // const float *range = rangeFinder[0]->getRangeImage();
    // currentRange = range[0] - 0.001908;
    
    // Get current wheel motor positions
    currentLW = rs[0]->getValue();
    currentRW = rs[1]->getValue();
    
    // Get current bearing
    // Code derived from Webots documentation
    // Available at https://cyberbotics.com/doc/reference/compass?tab-language=c++
    const double *north = compass[0]->getValues();
    double rad = atan2(north[0], north[2]);
    compassBearing = (rad - 1.5708) / M_PI * 180.0;
    if (compassBearing < 0.0){
      compassBearing = compassBearing + 360.0;
    }
    
    // Check if last step is nearly done
    stepNearlyDone = checkStepNearlyDone(currentLW,currentRW,nextLW,nextRW,compassBearing,forward,turning);
    
    // Check if the last step is complete
    lastStepComplete = checkStepComplete(currentLW,currentRW,nextLW,nextRW,compassBearing,currentRange,forward,turning);
    
    // If last step complete, load next step
    if(lastStepComplete == 1){   
      
      // Determine the action to take based on motion plan
      if(motionPlan[step] == 'F'){
        // Move forward one cell
        forward = 1;
        turning = 0;
        nextLW = currentLW + CELL_LENGTH_RAD;
        nextRW = currentRW + CELL_LENGTH_RAD;
        leftMotor->setVelocity(0.6*MAX_SPEED);
        rightMotor->setVelocity(0.6*MAX_SPEED);
      } else if (motionPlan[step] == 'L'){
        // Turn left
        turning = 1;
        forward = 0;
        nextLW = currentLW - CELL_ROTATION;
        nextRW = currentRW + CELL_ROTATION;
        leftMotor->setVelocity(0.4*MAX_SPEED);
        rightMotor->setVelocity(0.4*MAX_SPEED);
      } else if (motionPlan[step] == 'R'){
        // Turn left
        turning = 1;
        forward = 0;
        nextLW = currentLW + CELL_ROTATION;
        nextRW = currentRW - CELL_ROTATION;
        leftMotor->setVelocity(0.4*MAX_SPEED);
        rightMotor->setVelocity(0.4*MAX_SPEED);
      }
      
      // Determine heading from compass bearing
      if (compassBearing < 2 || compassBearing > 358){
        heading = 'N';
      } 
      else if (compassBearing > 88 && compassBearing < 92){
        heading = 'E';
      }
      else if (compassBearing > 178 && compassBearing < 182){
        heading = 'S';
      }
      else if (compassBearing > 268 && compassBearing < 272){
        heading = 'W';
      }
      
      // Determine row/column
      if (motionPlan[step-1] == 'F'){
        if(heading == 'N'){
          row--;
        } else if(heading == 'E'){
          col++;
        } else if(heading == 'S'){
          row++;
        } else if(heading == 'W'){
          col--;
        }
        
      }
      
      // Set motor positions for next step
      leftMotor->setPosition(nextLW);
      rightMotor->setPosition(nextRW);
      
      // Check for Surrounding Walls
      dsFValue = dsF[0]->getValue();
      // std::cout << "Front: " << dsFValue << std::endl;
      dsRValue = dsR[0]->getValue();
      // std::cout << "Right: " << dsRValue << std::endl;
      dsLValue = dsL[0]->getValue();
      // std::cout << "Left: " << dsLValue << std::endl;
      
      checkWalls(walls,dsFValue,dsRValue,dsLValue);
      fWall = walls[0];
      rWall = walls[1];
      lWall = walls[2];
      
      if(step > 0){
        // Print out the step, row, and column AND the walls surrounding it
        printStep(step,row,col,heading,lWall,fWall,rWall);
        
        motionExecution << (step-3) << "," 
                        << row << "," 
                        << col << "," 
                        << heading << "," 
                        << lWall << "," 
                        << fWall << "," 
                        << rWall << ",\n";
                        
        if (step == motionPlan.length()){
          motionExecution.close();
        }
      }
       
      step++;
      
    } 
    else if (stepNearlyDone == 1){
      leftMotor->setVelocity(0.05*MAX_SPEED);
      rightMotor->setVelocity(0.05*MAX_SPEED);
    }   
    
  };  
  
  std::cout << "[z5162711_MTRN4110_PhaseA] Motion plan executed!" << std::endl;
  delete robot;
  return 0;
}

