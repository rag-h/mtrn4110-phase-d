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
#include <ctime>
//#include "PhaseAFunctions.hpp"
#include "PhaseAFunctions.cpp"

#include <vector>

#define TIME_STEP 64


// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;


int main(int argc, char **argv) {
  time_t tstart, tend; 
  tstart = time(0);
  // create the Robot instance.
  Robot *robot = new Robot();
  
  const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
  
  PhaseAFunctions fun(robot);
  std::cout<<"[z5162972_MTRN4110_PhaseA] "<<"Reading in motion plan from" + MOTION_PLAN_FILE_NAME<<std::endl;
  vector<char> path = fun.getPath(MOTION_PLAN_FILE_NAME);  
  
  std::cout<<"[z5162972_MTRN4110_PhaseA] "<<"Executing motion plan..." <<std::endl;
  
  
    int start_x = 0;
    int start_y = 0;
    int goal_x = 4;
    int goal_y = 3;
    fun.initialiseFloodFill(start_x,start_y);
    fun.BFS(start_x, start_y,goal_x,goal_y,0);
    
    // fun.BFS(start_x, start_y,1,1,0);
    // fun.foundPath = 0;
    // fun.BFS(1,1,2,2,0);
    // fun.foundPath = 0; 
    // fun.BFS(2,2,3,3,0);
    // fun.foundPath = 0;
    // fun.BFS(3,3,4,4,0);
    // int currentExploredValue = 0;
    // int mazeValueChanged = 1;
    
  
  
 
  std::cout<<"[z5162972_MTRN4110_PhaseA] "<<"Motion plan executed!" <<std::endl;
  delete robot;
  
  tend = time(0); 
  cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< endl;
  return 0;
}




