// File:          keyboard_control.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>

using namespace webots;

// robot motor & sensor parameters
const int ROBOT_FORWARD_SPEED = 4;
const int ROBOT_TURNING_SPEED = 2;

// physical robot measurements
const int PI = 3.14159;   
const int WHEEL_RADIUS = 20;                            // mm
const int AXLE_LENGTH = 56.6;                           // mm
const int WHEEL_CIRCUM = 2*PI*WHEEL_RADIUS;             // mm
const int TURNING_CIRCUMFERENCE = 2*PI*(AXLE_LENGTH/2); // mm
 
// map/world parameters
const int TILE_SIZE = 165; // mm

// function definitions
void haltRobot(Motor *leftMotor, Motor *rightMotor);
void keyBoardDrive(int leftVelocity, int rightVelocity, Motor *leftMotor, Motor *rightMotor);

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Keyboard keyBoard;
  
  int timeStep = (int)robot->getBasicTimeStep();

  // add motors and sensors
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  PositionSensor *leftMotorSensor = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightMotorSensor = robot->getPositionSensor("right wheel sensor");

  // enable all motors and sensors
  keyBoard.enable(timeStep);
  leftMotorSensor->enable(timeStep);
  rightMotorSensor->enable(timeStep);
  leftMotor->setAcceleration(5);
  rightMotor->setAcceleration(5);
  //int leftMotorSpeed, rightMotorSpeed;

  while (robot->step(timeStep) != -1) {

    // read the keyboard input drive based on that direction
    int keyInput = keyBoard.getKey();
    if (keyInput == keyBoard.DOWN) {
      keyBoardDrive(-ROBOT_FORWARD_SPEED, -ROBOT_FORWARD_SPEED, leftMotor, rightMotor);
    }
    if (keyInput == keyBoard.UP) {
      keyBoardDrive(ROBOT_FORWARD_SPEED, ROBOT_FORWARD_SPEED, leftMotor, rightMotor);
    }
    if (keyInput == keyBoard.LEFT) {
      keyBoardDrive(0, ROBOT_TURNING_SPEED, leftMotor, rightMotor);
    }
    if (keyInput == keyBoard.RIGHT) {
      keyBoardDrive(ROBOT_TURNING_SPEED, 0, leftMotor, rightMotor);
    }
    if (keyInput == -1) {
      haltRobot(leftMotor, rightMotor);
    }
    std::cout << "pressing key: " << keyInput << std::endl;
    
    //leftMotor->setPosition(INFINITY);
    //rightMotor->setPosition(INFINITY);
  
    //leftMotor->setVelocity(leftVelocity);
    //rightMotor->setVelocity(rightVelocity);
  
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

// given positive/negtaive speed values for the left and right wheels, drive the vehicle
void keyBoardDrive(int leftVelocity, int rightVelocity, Motor *leftMotor, Motor *rightMotor) {
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(leftVelocity);
  rightMotor->setVelocity(rightVelocity);
}

// Stop the robot in it's current position
void haltRobot(Motor *leftMotor, Motor *rightMotor)
{
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
}





