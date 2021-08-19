#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <math.h>
#include <iomanip>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>


#define TIME_STEP 64
#define MAX_SPEED 6.28
#define CELL_LENGTH 0.165 
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.0566

#define PI 3.14159

#define WHEEL_RADIUS_MM 20                       //mm
#define WHEEL_CIRCUM 2*PI*WHEEL_RADIUS_MM        //mm
#define AXLE_LENGTH_MM 56.6                      //mm
#define TURNING_CIRCUM 2*PI*(AXLE_LENGTH_MM/2)   //mm

#define ROBOT_FORWARD_SPEED 4          // value between 0 and 6
#define ROBOT_TURNING_SPEED 2          // value between 0 and 6

#define TILE_SIZE 165 //mm

using namespace std;
using namespace webots;


class PhaseAFunctions {
  double prevTime = 0.0;
  Robot *robot;
  Motor *leftMotor;
  Motor *rightMotor;
  PositionSensor *leftMotorSensor;
  PositionSensor *rightMotorSensor;
  DistanceSensor *ds[4];
  char dsNames[4][10] = {"ds_front","ds_left", "ds_right","ds_back"};
  InertialUnit *imu;
  double rpy[3];
  double quat[4];
  
  double dsValues[8];
  char leftWall = 'N';
  char rightWall = 'N';
  char frontWall = 'N';
  char backWall = 'N';
  bool northWall = 0;
  bool southWall = 0;
  bool eastWall = 0;
  bool westWall = 0;
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
  double startPosLeft;
  double startPosRight;
  double map[11][19];
  
  int floodfill[9][5];
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
    void updateWalls();
    bool getNorthWall(){return this->northWall;}
    bool getSouthWall(){return this->southWall;}
    bool getEastWall(){return this->eastWall;}
    bool getWestWall(){return this->westWall;}
    char getHeading(){return this->currentDirection;}
    int foundPath = 0 ;
    //PID Functions
    double pid(double error);
    void initialiseFloodFill(int start_x,int start_y);
    void BFS(int start_x, int start_y,int goal_x,int goal_y,int currentExploredValue);
    void forward();
    void left();
    void right();
    void printFloodFill();
    bool turnRight(double startPosLeft, double startPosRight);
    bool turnLeft(double startPosLeft, double startPosRight);
    bool move_Forward();
    void haltRobot(Motor *leftMotor, Motor *rightMotor);
    void addWalls();
};
