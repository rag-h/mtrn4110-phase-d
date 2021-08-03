// File:          z5162966_MTRN4110_PhaseA.cpp
// Date:          09/06/2021
// Description:   Controller of E-puck for phase A - Driving and Perception
// Author:        Luke Jackson
// Modifications: 
// Platform:      Windows
// Notes:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <windows.h>

#define PI 3.14159

const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string CSV_OUTPUT_FILE = "../../MotionExecution.csv";
const std::string CONSOLE_PREFIX = "[z5162966_MTRN4110_PhaseA] ";

#define ROBOT_FORWARD_SPEED 4          // value between 0 and 6
#define ROBOT_TURNING_SPEED 2          // value between 0 and 6
#define DISTANCE_THRESHOLD 700 // value that the distance sensors read less than to trigger wall present

#define NUM_DIST_SENSORS 4
#define WHEEL_RADIUS 20                       //mm
#define WHEEL_CIRCUM 2*PI*WHEEL_RADIUS        //mm
#define AXLE_LENGTH 56.6                      //mm
#define TURNING_CIRCUM 2*PI*(AXLE_LENGTH/2)   //mm

#define TILE_SIZE 165 //mm


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// function definitions
bool moveForward(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor);
bool turnLeft(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor);
bool turnRight(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor);
void haltRobot(Motor *leftMotor, Motor *rightMotor);


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  PositionSensor *leftMotorSensor = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightMotorSensor = robot->getPositionSensor("right wheel sensor");
  DistanceSensor *ds[NUM_DIST_SENSORS];
  
  // setup distance sensors
  std::string dsNames[NUM_DIST_SENSORS];
  dsNames[0] = "dsF";
  dsNames[1] = "dsB";
  dsNames[2] = "dsL";
  dsNames[3] = "dsR";
  for (int i = 0; i < NUM_DIST_SENSORS; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(timeStep);
  }

  // Reading the motion plan from MotionPlan.txt file
  std::ifstream motionFileHandle(MOTION_PLAN_FILE_NAME);
  std::string motionPath;
  std::getline(motionFileHandle, motionPath);
  std::cout << CONSOLE_PREFIX << "Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
  std::cout << CONSOLE_PREFIX << "Motion Plan: " << motionPath << std::endl;
  std::cout << CONSOLE_PREFIX << "Motion plan read in!" << std::endl;
  std::cout << std::endl;
  motionFileHandle.close();  
  
  // array to store the movements, easier to iterate
  const int NUM_MOVES = motionPath.length() - 3;
  char motionArray [motionPath.length()];
  // populate movement array
  int iter = 0;
  for (char const &c: motionPath) {
    motionArray[iter] = c;
    iter++;
  }
  
  // opening csv file for outputting steps to
  std::ofstream output_csv(CSV_OUTPUT_FILE);
  std::string columnTitles[7] = {"Step", "Row", "Column", "Heading", "Left Wall", "Front Wall", "Right Wall"};
  // write column names to the csv file
  for(int i = 0; i < 7; i++)
  {
      output_csv << columnTitles[i];
      if (i != 6) output_csv << ",";
  }
  output_csv << "\n";
    
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  std::cout << CONSOLE_PREFIX << "Executing motion plan..." << std::endl;
  
  int currStep = 0, currRow = motionArray[0] - 48, currCol = motionArray[1] - 48;
  std::cout << (int)(motionArray[0]) << (int)(motionArray[1]) << std::endl;
  char currHeading = motionArray[2];
  int headings[4] = {'S','E','N','W'};
  int directionCounter = 0;
  
  std::string wallLeft = "N", wallFront = "N", wallRight = "Y";

  bool completedStep = true;
  
  double encoderOffsetLeft = 0;
  double encoderOffsetRight = 0;
  
  leftMotorSensor->enable(timeStep);
  rightMotorSensor->enable(timeStep);
  
  while (robot->step(timeStep) != -1){
    
    if (completedStep == true) {
      // output robot data to console
      std::cout << CONSOLE_PREFIX << "Step: " << std::setfill('0') << std::setw(3) << currStep;
      std::cout << ", Row: " << currRow << ", Column: " << currCol << ", Heading: " << currHeading;
      std::cout << ", Left Wall: " << wallLeft << ", Front Wall: " << wallFront << ", Right Wall: " << wallRight;
      std::cout << std::endl;
      //output robot data to csv file
      output_csv << currStep << "," << currRow << "," << currCol << "," << currHeading << "," << wallLeft << ",";
      output_csv << wallFront << "," << wallRight << "\n";
            
      // stop robot and re-zero position sensor offsets
      haltRobot(leftMotor, rightMotor);
      encoderOffsetLeft = leftMotorSensor->getValue();
      encoderOffsetRight = rightMotorSensor->getValue();
      
      if (currStep >= NUM_MOVES) break;
      completedStep = false;
      currStep++;
    }
        
    // Read the sensors and adjust wall presence strings
    double dsVals[NUM_DIST_SENSORS];
    for (int i = 0; i < NUM_DIST_SENSORS; i++) {
      dsVals[i] = ds[i]->getValue();
    }    
    // Process sensor data here to work out if wall is next to the robot
    if (dsVals[0] < DISTANCE_THRESHOLD) wallFront = "Y";
    else wallFront = "N";
    if (dsVals[2] < DISTANCE_THRESHOLD) wallLeft = "Y";
    else wallLeft = "N";
    if (dsVals[3] < DISTANCE_THRESHOLD) wallRight = "Y";
    else wallRight = "N";


    // Send actuator commands
    if (completedStep == false) {
      switch (motionArray[currStep + 2]) {
        case 'F':
          // Move Forward
          if (moveForward(encoderOffsetLeft, encoderOffsetRight, leftMotor, rightMotor, leftMotorSensor, rightMotorSensor) == true) {
            haltRobot(leftMotor, rightMotor);
            completedStep = true;
            // calculate position
            switch (directionCounter) {
              case 0:
                currRow++;
                break;
              case 1:
                currCol++;
                break;
              case 2:
                currRow--;
                break;
              case 3:
                currCol--;
                break;
            }
          
          }
          break;
        case 'L':
          // Turn Left
          if (turnLeft(encoderOffsetLeft, encoderOffsetRight, leftMotor, rightMotor, leftMotorSensor, rightMotorSensor) == true) {
            haltRobot(leftMotor, rightMotor);
            completedStep = true;
            directionCounter++;
          }
          break;
        case 'R':
          // Turn Right
          if (turnRight(encoderOffsetLeft, encoderOffsetRight, leftMotor, rightMotor, leftMotorSensor, rightMotorSensor) == true) {
            haltRobot(leftMotor, rightMotor);
            completedStep = true;
            directionCounter--;
          }
          break;
      }
    }
    
    // work out heading of the robot
    if (directionCounter > 3) directionCounter = 0;
    else if (directionCounter < 0) directionCounter = 3;
    currHeading = headings[directionCounter]; 
    
    
  }
  std::cout << CONSOLE_PREFIX << "Motion plan executed!"<< std::endl;
  
  output_csv.close();
  
  delete robot;
  return 0;
}




/*
    ---------- HELPER FUNCTIONS ----------
*/

// Moves the robot forward for one tile
// returns true when the bot has reached this position
// false otherwise
bool moveForward(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor) 
{
  double oneTileDist = TILE_SIZE/(WHEEL_CIRCUM)*2*PI;
  leftMotor->setPosition(startPosLeft + oneTileDist);
  leftMotor->setVelocity(ROBOT_FORWARD_SPEED);
  
  rightMotor->setPosition(startPosRight + oneTileDist);
  rightMotor->setVelocity(ROBOT_FORWARD_SPEED);
  
  if (leftMotorSensor->getValue() >= startPosLeft + oneTileDist) return true;
  else return false;
}

// Rotates the robot left 90 degrees
// returns true when the bot has reached this position
// false otherwise
bool turnLeft(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor) 
{
  double leftTurnDist = (TURNING_CIRCUM/(WHEEL_CIRCUM)*2*PI)/4;
  leftMotor->setPosition(startPosLeft - leftTurnDist);
  leftMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  rightMotor->setPosition(startPosRight + leftTurnDist);
  rightMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  if (rightMotorSensor->getValue() >= startPosRight + leftTurnDist) return true;
  else return false;
}

// Rotates the robot right 90 degrees
// returns true when the bot has reached this position
// false otherwise
bool turnRight(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor)
{
  double rightTurnDist = (TURNING_CIRCUM/(WHEEL_CIRCUM)*2*PI)/4;

  leftMotor->setPosition(startPosLeft + rightTurnDist);
  leftMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  rightMotor->setPosition(startPosRight - rightTurnDist);
  rightMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  if (leftMotorSensor->getValue() >= startPosLeft + rightTurnDist) return true;
  else return false;
}

// Stop the robot in it's current position
void haltRobot(Motor *leftMotor, Motor *rightMotor)
{
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
}


