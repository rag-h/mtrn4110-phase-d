// File: z5162972_MTRN4110_PhaseA.cpp
// Date: 3/06/2021
// Description:
// Author: Raghav Hariharan
// Modifications:
// Platform: Linux
// Notes: None

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>

//#include "PhaseAFunctions.hpp"
#include "PhaseAFunctions.cpp"

#include <vector>

#define TIME_STEP 64


// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
  
  PhaseAFunctions fun(robot);
  std::cout<<"[z5162972_MTRN4110_PhaseA] "<<"Reading in motion plan from" + MOTION_PLAN_FILE_NAME<<std::endl;
  vector<char> path = fun.getPath(MOTION_PLAN_FILE_NAME);  
  
  int pathPosition = 3;
  bool motionStarted = 0;
  
  
  std::string s(path.begin(), path.end());
  std::cout <<"[z5162972_MTRN4110_PhaseA] "<< "Motion Plan: " <<s <<"..."<<std::endl;
  std::cout <<"[z5162972_MTRN4110_PhaseA] "<< "Motion plan read in!" <<std::endl;  
  std::cout<<"[z5162972_MTRN4110_PhaseA] "<<"Executing motion plan..." <<std::endl;
  
  
  while (robot->step(TIME_STEP) != -1){    
  //std::cout<<"x: " << gyro->getValues()[0]<< " y: " << gyro->getValues()[1]<< " z: "<< gyro->getValues()[2]<<std::endl;
     if(pathPosition == 3 && motionStarted == 0){
       fun.printCurrentPosition();
       motionStarted = 1;
     }
     
     if(pathPosition < path.size()){
       if(path[pathPosition] == 'F'){
        pathPosition = pathPosition + fun.moveForward();
       }else{
         pathPosition = pathPosition + fun.turn(path[pathPosition]);
       }
     }else{
       break;
     }
  }
  fun.generateCSV();
  std::cout<<"[z5162972_MTRN4110_PhaseA] "<<"Motion plan executed!" <<std::endl;
  delete robot;
  return 0;
}