// File:          KeyBoardControl.cpp
// Date:          11/08/2021
// Description:   Control the robot with the keyboard if requested
// Author:        Luke Jackson
// Modifications: 
// Platform:      Windows
// Notes:

//#include <webots/Motor.hpp>
//#include <webots/DistanceSensor.hpp>
//#include <webots/PositionSensor.hpp>
//#include <webots/Robot.hpp>

#include <iostream>
#include <string>

#include "createPath.cpp"
//#include "followPath.cpp"

//using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  /*
  // create the Robot instance.
  Robot *robot = new Robot();

  int timeStep = (int)robot->getBasicTimeStep();

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  PositionSensor *leftMotorSensor = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightMotorSensor = robot->getPositionSensor("right wheel sensor");
  DistanceSensor *ds[NUM_DIST_SENSORS];
  */

  // setup distance sensors
  /*
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
  */

  // create path plan from current position


  //while (robot->step(timeStep) != -1) {
    
  //};

  // Enter here exit cleanup code.

  cout << "Beginning program" << endl;
  vector<string> temp = createFirstPath();
  for (string t: temp) {
    cout << t << endl;
  }
  cout << "Ending program" << endl;

  //delete robot;
  return 0;
}
