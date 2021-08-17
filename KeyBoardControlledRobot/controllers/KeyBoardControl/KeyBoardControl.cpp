// File:          KeyBoardControl.cpp
// Date:          11/08/2021
// Description:   Control the robot with the keyboard if requested
// Author:        Luke Jackson
// Modifications: 
// Platform:      Windows
// Notes:

#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>

#include <iostream>
#include <string>

#include "createPath.cpp"
#include "followPath.cpp"

using namespace webots;
using namespace std;

// function defintions
void keyBoardDrive(int leftVelocity, int rightVelocity, Motor *leftMotor, Motor *rightMotor);

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Keyboard keyBoard;

  int timeStep = (int)robot->getBasicTimeStep();

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

  double encoderOffsetLeft = 0;
  double encoderOffsetRight = 0;

  leftMotorSensor->enable(timeStep);
  rightMotorSensor->enable(timeStep);
  keyBoard.enable(timeStep);

  // create initial path plan from current and desired position
  vector<int> startPoint{0,0};
  vector<int> endPoint{3,0};
  vector<char> movingPath = createNewPath(startPoint, SOUTH, endPoint);
  int currStep = -1;
  bool completedStep = true;

  cout << "Beginning movement" << endl;

  bool autoMove = true;

  while (robot->step(timeStep) != -1) {
    // if control key pressed then switch states
    int keyInput = keyBoard.getKey();
    if (keyInput == keyBoard.CONTROL || keyInput == keyBoard.ALT) {
      autoMove = !autoMove;
      if (autoMove) {
        cout << "SWITCHING TO AUTONOMOUS MODE" << endl;
      } else {
        cout << "SWITCHING TO DRIVER MODE" << endl;
      }
    }

    // do automatic movement steps
    if (autoMove) {
      if (completedStep == true) {
        // stop robot and re-zero position sensor offsets
        haltRobot(leftMotor, rightMotor);
        encoderOffsetLeft = leftMotorSensor->getValue();
        encoderOffsetRight = rightMotorSensor->getValue();
        
        if (currStep > (int)movingPath.size()) {
          cout << "destination reached" << endl;
          break;
        }
        completedStep = false;
        currStep++;
      }
      if (completedStep == false) {
        moveNextStep(movingPath[currStep], completedStep, encoderOffsetLeft, encoderOffsetRight,leftMotor, rightMotor, leftMotorSensor, rightMotorSensor);
      }
    }  

    // if DRIVER mode
    if (!autoMove) {
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
    }

  };
  cout << "Motion ended" << endl;
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