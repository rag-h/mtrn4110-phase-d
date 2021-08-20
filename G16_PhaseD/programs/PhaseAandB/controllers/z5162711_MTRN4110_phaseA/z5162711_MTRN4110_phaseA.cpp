// File:          z5162711_MTRN4110_phaseA.cpp
// Date:          18/08/2021
// Description:   Path Finding and E-Puck Controller
// Author:        Phase A: James Jonathon Davies, Phase B: Luke Jackson
// Modifications:
// Platform:      MacOS

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/RangeFinder.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <deque>
#include <vector>
#include <queue>
#include <algorithm>

#include "path.cpp"
#include "graph.cpp"
#include "PhaseAFunctions.cpp"
#include "PhaseBFunctions.cpp"

using namespace webots;
using namespace std;
const string MAP_FILE_NAME = "../../../../MapBuilt.txt";

int main(int argc, char **argv) {
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// Phase B
  
  // Read in the map text file
  ifstream mapFileHandle(MAP_FILE_NAME); // open the file
  string lineTemp;
  char map[ROWS][CHARS_PER_LINE]; // 11 rows of 38 columns
  cout << CONSOLE_PREFIX << "Reading in map from ../../../../MapBuilt.txt..." << endl;
  // get the characters one at a time
  for (int i = 0; i < ROWS; i++) {
      getline(mapFileHandle, lineTemp);
      for (int j = 0; j < CHARS_PER_LINE; j++) {
          map[i][j] = lineTemp[j];
      }
  }
  // print out the map text file
  for (int i = 0; i < ROWS; i++) {
      cout << CONSOLE_PREFIX;
      for (int j = 0; j < CHARS_PER_LINE; j++) {
          cout << map[i][j];
      }
      cout << endl;
  }
  cout << CONSOLE_PREFIX << "Map read in!" << endl;
  mapFileHandle.close();

  // find coord of start and end positon
  int orientation = SOUTH;
  int rowCoord = 0;
  int startLoc = 0;
  int endLoc = 0;
  vector<int> startPosGlobal;
  for (int row = 1; row < ROWS; row += 2) {
      int colCoord = 0;
      for (int col = 2; col < CHARS_PER_LINE; col += SPACE_BETWEEN_CENTERS) {
          int currPos = (rowCoord * NUM_COLS) + colCoord;
          // finding if start or end point
          if (isStartPoint(map[row][col])) {
              if (map[row][col] == 'v') orientation = SOUTH;
              else if (map[row][col] == '>') orientation = EAST;
              else if (map[row][col] == '<') orientation = WEST;
              else if (map[row][col] == '^') orientation = NORTH;
              startPosGlobal.push_back(row);
              startPosGlobal.push_back(col);
              startLoc = currPos;
          }
          if (map[row][col] == 'x') endLoc = currPos;
          colCoord++;
      }
      rowCoord++;
  }
  // convert start and end postion to coordainates (row, column)
  vector<int> startPos = positionToCoord(startLoc);
  vector<int> endPos = positionToCoord(endLoc);
  // flood fill
  // initialize array and fill with high number
  int mapArray[NUM_ROWS][NUM_COLS];
  for (int i = 0; i < NUM_ROWS; i++) for (int j = 0; j < NUM_COLS; j++) mapArray[i][j] = NUM_ROWS * NUM_COLS;
  // begin flood fill algorithm
  mapArray[endPos[0]][endPos[1]] = 0;
  int currExploredValue = 0;
  int mazeValueChanged = 1;
  // loop through all values
  while (mazeValueChanged != 0) {
      mazeValueChanged = 0;
      for (int i = 0; i < NUM_ROWS; i++) {
          for (int j = 0; j < NUM_COLS; j++) {
              if (mapArray[i][j] == currExploredValue) {
                  char walls[4]; // North, East, South, West
                  // find wall characters
                  walls[0] = map[i * 2][j * SPACE_BETWEEN_CENTERS + 2];
                  walls[1] = map[i * 2 + 1][j * SPACE_BETWEEN_CENTERS + 4];
                  walls[2] = map[i * 2 + 2][j * SPACE_BETWEEN_CENTERS + 2];
                  walls[3] = map[i * 2 + 1][j * SPACE_BETWEEN_CENTERS];
                  for (int k = 0; k < 4; k++) {
                      if (walls[k] == ' ') { // if wall does not exist
                          switch (k) 
                          {
                          case 0:
                              if (mapArray[i - 1][j] == NUM_ROWS * NUM_COLS) {
                                  mapArray[i - 1][j] = mapArray[i][j] + 1;
                                  mazeValueChanged = 1;
                              }
                              break;
                          case 1:
                              if (mapArray[i][j + 1] == NUM_ROWS * NUM_COLS) {
                                  mapArray[i][j + 1] = mapArray[i][j] + 1;
                                  mazeValueChanged = 1;
                              }
                              break;
                          case 2:
                              if (mapArray[i + 1][j] == NUM_ROWS * NUM_COLS) {
                                  mapArray[i + 1][j] = mapArray[i][j] + 1;
                                  mazeValueChanged = 1;
                              }
                              break;
                          case 3:
                              if (mapArray[i][j - 1] == NUM_ROWS * NUM_COLS) {
                                  mapArray[i][j - 1] = mapArray[i][j] + 1;
                                  mazeValueChanged = 1;
                              }
                              break;
                          }
                      }
                  }
              }
          }
      }
      currExploredValue++;
  }
  
  // Path Planning alogrithm
  // run through flood fill array constructing a graph 
  // this graph has adjaceny list representation,
  // then BFS this graph to retrieve all shortest paths
  // the convert these paths from array indices to coords
  
  // list of paths contains a list of coordaintes
  // list of coordinates example: [0,0]
  //[[[0,0], [0,1], ..], (new path:) [[1,1], [0,1], ..]
  cout << CONSOLE_PREFIX << "Finding shortest paths..." << endl;
  Graph g(NUM_ROWS, NUM_COLS);
  g.graphFF(NUM_ROWS, NUM_COLS, mapArray, ROWS, CHARS_PER_LINE, map);
  vector<vector<int> > newAdjList = g.getAdjList();
  vector<vector<vector<int> > > allPaths = getPathCoords(newAdjList, startLoc, endLoc, NUM_ROWS, NUM_COLS);

  // Find the path with fewest turns
  int pathCounter = 1;
  for (vector<vector<int> > path : allPaths) {
      cout << CONSOLE_PREFIX << "Path - " << pathCounter << ":" << endl;
      reverse(path.begin(), path.end());
      printMapFromCoords(map, path, startPosGlobal);
  }
  cout << CONSOLE_PREFIX << pathCounter - 1 << " shortest paths found!" << endl;
  // Find the minimum number of turns:
  // loop through all paths, create String movements for all, shortest string
  // is the shortest path
  cout << CONSOLE_PREFIX << "Finding shortest path with least turns..." << endl;
  vector<char> shortestPathString;
  int shortestPathLength = NUM_ROWS * NUM_COLS * 3;
  int shortestPathIndex = 0;
  int counter = 0;

  for (vector<vector<int> > path : allPaths) {
      vector<char> pathString;
      int length = pathToString(pathString, path, orientation, endPos, startPos);
      if (length < shortestPathLength) {
          shortestPathString.clear();
          shortestPathLength = length;
          shortestPathIndex = counter;
          // copy string to save
          for (char chars: pathString) {
              shortestPathString.push_back(chars);
          }
      }
      counter++;
  }
  reverse(allPaths[shortestPathIndex].begin(), allPaths[shortestPathIndex].end());
  printMapFromCoords(map, allPaths[shortestPathIndex], startPosGlobal);

  cout << CONSOLE_PREFIX << "Shortest path with least turns found!" << endl;
  cout << CONSOLE_PREFIX << "Path Plan (" << shortestPathLength << " steps): ";
  for (int step: shortestPathString) {
      cout << (char)step;
  }
  cout << endl;
  // - perform simulation steps until Webots is stopping the controller


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// Phase A 

  // Read motion plan
  // initialise string for motion plan
  std::string motionPlan;        
  for (char c: shortestPathString) {
      motionPlan.append(1, c);
  }
  
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
  
  std::cout << CONSOLE_PREFIX << "Executing path plan..." << std::endl;

  // Initialise Variables
  int step = 3;
  char rowChar = motionPlan[0];
  char colChar = motionPlan[1];
  int row = (int)rowChar - 48;;
  int col = (int)colChar - 48;;
  char heading = motionPlan[2];
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
  
  // MAIN LOOP    
  while (robot->step(TIME_STEP) != -1 && step < (motionPlan.length()+1)){
    
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
      dsRValue = dsR[0]->getValue();
      dsLValue = dsL[0]->getValue();
      checkWalls(walls,dsFValue,dsRValue,dsLValue);
      fWall = walls[0];
      rWall = walls[1];
      lWall = walls[2];
      
      if(step > 0){
        // Print out the step, row, and column AND the walls surrounding it
        printStep(step,row,col,heading,lWall,fWall,rWall);
      } 
      step++;
    } 
    else if (stepNearlyDone == 1){
      leftMotor->setVelocity(0.05*MAX_SPEED);
      rightMotor->setVelocity(0.05*MAX_SPEED);
    }   
  };  
  std::cout << CONSOLE_PREFIX << "Motion plan executed!" << std::endl;
  delete robot;
  return 0;
}

