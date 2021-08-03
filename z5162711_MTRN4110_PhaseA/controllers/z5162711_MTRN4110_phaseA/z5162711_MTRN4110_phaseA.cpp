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
#include <fstream>
#include <string>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <sstream>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define PI 3.14193
#define MOTION_PLAN_FILE_NAME "../../MotionPlan.txt"
#define CELL_LENGTH_RAD 8.25 
//#define CELL_ROTATION 2.2227
#define CELL_ROTATION 2.232

#define TOL1 0.4
#define TOL2 0.009

using namespace webots;

// Function Definitions
int checkStepComplete(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing, double currentRange, int forward, int turning);
int checkStepNearlyDone(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing,int forward, int turning);
void checkWalls(char* walls, int dsFValue, int dsRValue, int dsLValue);
void printStep(int step, int row, int col, char heading, char lWall, char fWall, char rWall);
void writeToCSV(int step, int row, int col, char heading, char lWall, char fWall, char rWall);

int main(int argc, char **argv) {

  // Read motion plan
  std::cout << "[z5162711_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
  
  std::ifstream f;               // create file stream
  f.open(MOTION_PLAN_FILE_NAME); // open file and connect to file stream
  std::string motionPlan;        // initialise string for motion plan
  std::getline(f, motionPlan);   // get motion plan string from stream
  
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
      
      // std::cout << "Step Done" << std::endl;
      // std::cout << "current bearing: " << compassBearing << std::endl;
      // std::cout << "Range: " << currentRange << std::endl;
      
      
      
      
      
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

int checkStepComplete(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing, double currentRange, int forward, int turning){
  
  // std::cout << "currentLW: " << currentLW << std::endl;
  // std::cout << "nextLW: " << nextLW << std::endl;
  // std::cout << "currentRW: " << currentRW << std::endl;
  // std::cout << "nextRW: " << nextRW << std::endl;
  // std::cout << "currentRange: " << currentRange << std::endl;
  // std::cout << "compassBearing: " << compassBearing << std::endl;
  // std::cout << "LW Diff: " << abs(currentLW - nextLW) << std::endl;
  // std::cout << "RW Diff: " << abs(currentRW - nextRW) << std::endl;
  
  if(abs(currentLW - nextLW) > TOL2){
    return 0;
  } else if (abs(currentRW - nextRW) > TOL2){
    return 0;
  } 
  return 1;
  
}

int checkStepNearlyDone(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing, int forward, int turning){
  
  if(abs(currentLW - nextLW) > TOL1){
    return 0;
  } else if (abs(currentRW - nextRW) > TOL1){
    return 0;
  } 
  return 1;
  
}

void checkWalls(char* walls, int dsFValue, int dsRValue, int dsLValue){
  
  char lWall = 'N';
  char rWall = 'N';
  char fWall = 'N';
  
  if(dsFValue > 600){
    fWall = 'Y';
  } else {
    fWall = 'N';
  }
  walls[0] = fWall;
  
  if(dsRValue > 600){
    rWall = 'Y';
  } else {
    rWall = 'N';
  }
  walls[1] = rWall;
  
  if(dsLValue > 600){
    lWall = 'Y';
  } else {
    lWall = 'N';
  }
  walls[2] = lWall;

}

void printStep(int step, int row, int col, char heading, char lWall, char fWall, char rWall){
  std::cout << "[z5162711_MTRN4110_PhaseA] Step: ";
  if (step < 13){ 
    std::cout << "00"; 
  }
  if (step >= 13 && step < 103){ 
    std::cout << "0"; 
  }
    std::cout << (step-3)
              << ", Row: " << row
              << ", Column: " << col
              << ", Heading: " << heading
              << ", Left Wall: " << lWall
              << ", Front Wall: " << fWall
              << ", Right Wall: " << rWall
              << std::endl;
  
}