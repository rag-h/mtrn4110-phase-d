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
using namespace std;

// function definitions
void moveNextStep(char nextDir, bool &completedStep, double encoderOffsetLeft, double encoderOffsetRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor);
bool moveForward(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor);
bool turnLeft(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor);
bool turnRight(double startPosLeft, double startPosRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor);
void haltRobot(Motor *leftMotor, Motor *rightMotor);

bool completedStep = true;

// moves the next direction specified in the move list
void moveNextStep(char nextDir, bool &completedStep, double encoderOffsetLeft, double encoderOffsetRight, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftMotorSensor, PositionSensor *rightMotorSensor) {
  // Send actuator commands
  switch (nextDir) {
    case 'F':
      // Move Forward
      if (moveForward(encoderOffsetLeft, encoderOffsetRight, leftMotor, rightMotor, leftMotorSensor, rightMotorSensor) == true) {
        haltRobot(leftMotor, rightMotor);
        completedStep = true;    
      }
      break;
    case 'L':
      // Turn Left
      if (turnLeft(encoderOffsetLeft, encoderOffsetRight, leftMotor, rightMotor, leftMotorSensor, rightMotorSensor) == true) {
        haltRobot(leftMotor, rightMotor);
        completedStep = true;
      }
      break;
    case 'R':
      // Turn Right
      if (turnRight(encoderOffsetLeft, encoderOffsetRight, leftMotor, rightMotor, leftMotorSensor, rightMotorSensor) == true) {
        haltRobot(leftMotor, rightMotor);
        completedStep = true;
      }
      break;
  }
}

 
/*
    ---------- MOVEMENT FUNCTIONS ----------
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


