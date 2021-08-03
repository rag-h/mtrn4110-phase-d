#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <math.h>
#include <iomanip>
#include <webots/InertialUnit.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define CELL_LENGTH 0.165 
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.0566

using namespace std;
using namespace webots;


class PhaseAFunctions {
  double prevTime = 0.0;
  Robot *robot;
  Motor *leftMotor;
  Motor *rightMotor;
  DistanceSensor *ds[3];
  char dsNames[3][10] = {"ds_front","ds_left", "ds_right"};
  InertialUnit *imu;
  double rpy[3];
  double quat[4];
  
  double dsValues[8];
  char leftWall = 'N';
  char rightWall = 'N';
  char frontWall = 'N';
  int currentCol;
  int currentRow;
  char currentDirection;
  int stepNumber = 0;
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  ofstream outputFile;
  
  //VARIABLES FOR THE PID
  double prevError = 0.0;
  double prop = 0.0;
  double intg = 0.0;
  double diff = 0.0;
  double counter = 0.0;
  double ki = 0.0;
  double kp = 0.005;
  double kd = 0.15;
  double expectedYaw = 0;
  public:
    PhaseAFunctions(Robot *_robot);
    ~PhaseAFunctions(){};
    vector<char> getPath(string motionPlan);
    vector<string> motionExecution;
    int moveForward();
    int turn(char s);
    void setMotors();
    void printCurrentPosition();
    void updatePosition(char movement);
    void getDistanceSensorReadings();
    int getImu();
    void generateCSV();
    
    //PID Functions
    double pid(double error);
};