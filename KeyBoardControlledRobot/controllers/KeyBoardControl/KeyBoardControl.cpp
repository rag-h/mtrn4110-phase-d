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
#include <webots/Compass.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <chrono>
#include <math.h>
#include <iostream>
#include <string>

#include "createPath.cpp"
#include "followPath.cpp"

using namespace webots;
using namespace std;

#define SPACEBAR 32
#define DEBOUNCE_TIME 500.0 // milliseconds
const int TILE_CENTRE_DISTANCE = 650;

// function defintions
void keyBoardDrive(int leftVelocity, int rightVelocity, Motor *leftMotor, Motor *rightMotor);
directions getOrientation(Compass *compass);
void realignSouth(Motor *leftMotor, Motor *rightMotor);
void stepForward(Motor *leftMotor, Motor *rightMotor);

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Keyboard keyBoard;

  int timeStep = (int)robot->getBasicTimeStep();

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  PositionSensor *leftMotorSensor = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightMotorSensor = robot->getPositionSensor("right wheel sensor");
  Compass *compass = robot->getCompass("compass");
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
  double encoderLeftTotal = 0;
  double encoderRightTotal = 0;

  leftMotorSensor->enable(timeStep);
  rightMotorSensor->enable(timeStep);
  keyBoard.enable(timeStep);
  compass->enable(timeStep);

  // create initial path plan from current and desired position
  vector<int> startPoint{0,0};
  vector<int> endPoint{2,4};
  vector<char> movingPath = createNewPath(startPoint, SOUTH, endPoint);
  int currStep = -1;
  bool completedStep = true;

  cout << "Beginning movement" << endl;

  bool autoMove = true; // set to true to begin in autonomous mode

  auto prevModeTime = chrono::high_resolution_clock::now();
  bool realignOrientation = false;
  bool realignMovementForward = false;
  bool rotateLeft = false;
  bool realignMovementSide = false;
  
  while (robot->step(timeStep) != -1) {
    // if control key pressed then switch states
    int keyInput = keyBoard.getKey();
    
    if (keyInput == SPACEBAR) {
      // check that debounce time has passed
      double elapsedTime = chrono::duration<double, std::milli>(chrono::high_resolution_clock::now()-prevModeTime).count();
      if (elapsedTime > DEBOUNCE_TIME) {
        prevModeTime = chrono::high_resolution_clock::now();
        autoMove = !autoMove;
        if (autoMove) {
          cout << "SWITCHING TO AUTONOMOUS MODE" << endl;
          // reset map path to take
          realignOrientation = true;
          cout << "REALIGNING, PLEASE WAIT ..." << endl;          
        } else {
          cout << "SWITCHING TO DRIVER MODE" << endl;
        }
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

    // calculate current position, assumed to start at the beginning
    //encoderLeftTotal += leftMotorSensor->getValue();
    //encoderRightTotal += leftMotorSensor->getValue();
    //cout << "Left = " << encoderLeftTotal << " Right = " << encoderOffsetRight << endl;


    // realign orientation and position to centre of tile
    if (realignOrientation) {
      //cout << "realigning south" << endl;
      // realign orientation of robot to South
      if (getOrientation(compass) != SOUTH) {
        realignSouth(leftMotor, rightMotor);
      } else {
        realignOrientation = false;
        realignMovementForward = true;
      }
    }
    if (realignMovementForward) {
      //cout << "realigning forward - " << ds[0]->getValue() << endl;
      // realign to centre of tile
      // drive forward until X distance away from wall
      if (ds[0]->getValue() > TILE_CENTRE_DISTANCE) {
        stepForward(leftMotor, rightMotor);
      } else {
        realignMovementForward = false;
        rotateLeft = true;
      }      
    }
    if (rotateLeft) {
      // turn to face left (west)
      //cout << "rotating left" << endl;
      if (getOrientation(compass) != WEST) {
        realignSouth(leftMotor, rightMotor);
      } else {
        rotateLeft = false;
        realignMovementSide = true;
      }
    }
    if (realignMovementSide) {
      //cout << "realigning forward again" << endl;
      // realign to centre of tile
      // drive forward until X distance away from wall
      if (ds[0]->getValue() > TILE_CENTRE_DISTANCE) {
        stepForward(leftMotor, rightMotor);
      } else {
        realignMovementSide = false;
        cout << "finished realignment, plotting new course" << endl;
        haltRobot(leftMotor, rightMotor);
        vector<int> currPos{4,1};
        movingPath = createNewPath(currPos, WEST, endPoint);
        cout << "new course: ";
        for (char c: movingPath) {
          cout << c;
        } 
        cout << endl;
        encoderOffsetLeft = leftMotorSensor->getValue();
        encoderOffsetRight = rightMotorSensor->getValue();
        currStep = -1;
        completedStep = true;
      }
    }

    // rotate to face East, drive until X distance away from wall)





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

directions getOrientation(Compass *compass) {
  double tolerance = 0.5;
  double target = 1000.0;
  double compass_0 = compass->getValues()[0] * 1000.0;
  double compass_2 = compass->getValues()[2] * 1000.0;
  if (compass_0 > (target - tolerance) && compass_0 < (target + tolerance)) {
    //cout << "Facing North" << endl;
    return NORTH;
  } else if (compass_0 > (-target - tolerance) && compass_0 < (-target + tolerance)) {
    //cout << "Facing South" << endl;
    return SOUTH;
  } else if (compass_2 > (target - tolerance) && compass_2 < (target + tolerance)) {
    //cout << "Facing West" << endl;
    return WEST;
  } else if (compass_2 > (-target - tolerance) && compass_2 < (-target + tolerance)) {
    //cout << "Facing East" << endl;
    return EAST;
  }
  return UNKNOWN;
}

void realignSouth(Motor *leftMotor, Motor *rightMotor) {
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(ROBOT_TURNING_SPEED);
  rightMotor->setVelocity(-ROBOT_TURNING_SPEED);
}

void stepForward(Motor *leftMotor, Motor *rightMotor) {
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(ROBOT_FORWARD_SPEED);
  rightMotor->setVelocity(ROBOT_FORWARD_SPEED);
}