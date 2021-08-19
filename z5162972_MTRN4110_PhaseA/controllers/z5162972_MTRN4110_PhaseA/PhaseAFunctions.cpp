#include "PhaseAFunctions.hpp"



bool PhaseAFunctions::move_Forward()
{
  // std::cout<<"FORWARD"<<std::endl;
  double oneTileDist = TILE_SIZE/(WHEEL_CIRCUM)*2*PI;
  leftMotor->setPosition(startPosLeft + oneTileDist);
  leftMotor->setVelocity(ROBOT_FORWARD_SPEED);
  
  rightMotor->setPosition(startPosRight + oneTileDist);
  rightMotor->setVelocity(ROBOT_FORWARD_SPEED);
  
  std::cout<<startPosRight << " "<<startPosLeft + oneTileDist <<std::endl;
  
  // if (abs(leftMotorSensor->getValue() - startPosLeft - oneTileDist)< 0.01) return true;
  if (leftMotorSensor->getValue() >= startPosLeft + oneTileDist) return true;
  else if(abs(leftMotorSensor->getValue() - startPosLeft - oneTileDist)< 0.01) return true;
  else return false;
  
  return false;
}

// Rotates the robot left 90 degrees
// returns true when the bot has reached this position
// false otherwise
bool PhaseAFunctions::turnLeft(double startPosLeft, double startPosRight)
{
  std::cout<<"LEFT"<<std::endl;
  double leftTurnDist = (TURNING_CIRCUM/(WHEEL_CIRCUM)*2*PI)/4;
  leftMotor->setPosition(startPosLeft - leftTurnDist);
  leftMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  rightMotor->setPosition(startPosRight + leftTurnDist);
  rightMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  if (rightMotorSensor->getValue() >= startPosRight + leftTurnDist) return true;
  else if(abs(rightMotorSensor->getValue() - startPosRight - leftTurnDist)< 0.01) return true;
  else return false;
  
  return false;
}

// Rotates the robot right 90 degrees
// returns true when the bot has reached this position
// false otherwise
bool PhaseAFunctions::turnRight(double startPosLeft, double startPosRight)
{
  std::cout<<"RIGHT"<<std::endl;
  double rightTurnDist = (TURNING_CIRCUM/(WHEEL_CIRCUM)*2*PI)/4;

  leftMotor->setPosition(startPosLeft + rightTurnDist);
  leftMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  rightMotor->setPosition(startPosRight - rightTurnDist);
  rightMotor->setVelocity(ROBOT_TURNING_SPEED);
  
  if (leftMotorSensor->getValue() >= startPosLeft + rightTurnDist) return true;
  else if(abs(leftMotorSensor->getValue() - startPosLeft - rightTurnDist)< 0.01) return true;
  else return false;
  
  return false;
}

// Stop the robot in it's current position
void PhaseAFunctions::haltRobot(Motor *leftMotor, Motor *rightMotor)
{
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
}




PhaseAFunctions::PhaseAFunctions(Robot *_robot){
  this->robot = _robot;
  
  this->leftMotor = _robot->getMotor("left wheel motor");
  this->rightMotor = _robot->getMotor("right wheel motor");
  this->leftMotorSensor = _robot->getPositionSensor("left wheel sensor");
  this->rightMotorSensor = _robot->getPositionSensor("right wheel sensor");
  this->leftMotor->setPosition(INFINITY);
  this->rightMotor->setPosition(INFINITY);
  this->leftMotor->setVelocity(0.0);
  this->rightMotor->setVelocity(0.0);
  this->imu = robot->getInertialUnit("imu");
  this->imu->enable(TIME_STEP);  
  
  this->leftMotorSensor = robot->getPositionSensor("left wheel sensor");
  this->rightMotorSensor = robot->getPositionSensor("right wheel sensor");
  
  for (int i = 0; i < 4; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);    
    std::cout<< " " <<std::endl;
  }
  
  
  leftMotorSensor->enable(TIME_STEP);
  rightMotorSensor->enable(TIME_STEP);
 
 
  for (int j = 0; j < 11;j++){
    for(int k = 0; k < 19; k++){
    this->map[j][k] = 0;
    }
  }
  for(int k = 0; k < 11; k++){
        this->map[k][0] = 1;
        this->map[k][18] = 1;
  }
  
    for(int j = 0; j < 19; j++){
        this->map[0][j] = 1;
        this->map[10][j] = 1;
  }
}


vector<char> PhaseAFunctions::getPath(string motionPlan){
  vector<char> path;
      ifstream my_file (motionPlan);
      if (my_file.is_open()){
        char ch;
        while (1) {
          my_file >> ch;
          if (my_file.eof())
          break;
          path.push_back(ch);
          //std::cout << ch<<std::endl;
        }
      }else{
        std::cout<<"No such File"<<std::endl;
      }
      
      my_file.close();
      
      //Setting the current position and the current direction
      this->currentRow = path[0]-'0';
      this->currentCol = path[1]-'0';
      this->currentDirection = path[2];
      
      if(currentDirection == 'N'){
   expectedYaw = 0.00;
 }else if(currentDirection == 'S'){
  expectedYaw = -180.00;
 }else if(currentDirection == 'E'){
   expectedYaw = -90.00;
   
 }else if(currentDirection == 'W'){
   expectedYaw = 90.00;
 }
      
      return path;
};

void PhaseAFunctions::setMotors(){
    
    //std::cout<< leftSpeed << " " << rightSpeed << std::endl;
    
  leftMotor->setVelocity(leftSpeed);
  rightMotor->setVelocity(rightSpeed);
}

int PhaseAFunctions::getImu(){
  this->rpy[0] = this->imu->getRollPitchYaw()[0] * (180 / M_PI);
  this->rpy[1] = this->imu->getRollPitchYaw()[1] * (180 / M_PI);
  this->rpy[2] = this->imu->getRollPitchYaw()[2] * (180 / M_PI);
  //std::cout<< "roll: " <<rpy[0]  <<" pitch: " << rpy[1] << " yaw: " << rpy[2] <<std::endl;
  return 0;
}

int PhaseAFunctions::moveForward(){
  getImu();
  getDistanceSensorReadings();
  
  if(robot->getTime() >= (prevTime + CELL_LENGTH/(WHEEL_RADIUS*MAX_SPEED)) || dsValues[0] < 850)  {
    leftSpeed = 0.0;
    rightSpeed = 0.0;
    prevTime = robot->getTime();
    setMotors();
    updatePosition('F');
    printCurrentPosition();
    return 1;
  }
  
  leftSpeed = MAX_SPEED;
  rightSpeed = MAX_SPEED;
  setMotors();
  
  
  
  
  return 0;
}

int PhaseAFunctions::turn(char s){
  getImu();
  
   if(currentDirection == 'N'){
     if(s == 'L'){
     expectedYaw = 90.00;
   }else if(s == 'R'){
     expectedYaw = -90.00;
   }
   }else if(currentDirection == 'S'){
    if(s == 'L'){
     expectedYaw = -90.00;
   }else if(s == 'R'){
     expectedYaw = 90.00;
   }
   }else if(currentDirection == 'E'){
    if(s == 'L'){
     expectedYaw = 0.00;
   }else if(s == 'R'){
     expectedYaw = -180.00;
   }
   }else if(currentDirection == 'W'){
      if(s == 'L'){
     expectedYaw = 180.00;
   }else if(s == 'R'){
     expectedYaw = 0.00;
   }
   }


  double turnError = abs((rpy[2] - expectedYaw)) * (M_PI/180);
  double turnKp = 1.5;
  //std::cout <<"error: " << abs(turnError) <<std::endl;
  if(robot->getTime() >= (prevTime + 0.5)){
    if(s == 'L'){
    leftSpeed  = - min(turnKp*turnError, 0.2 * MAX_SPEED);
    rightSpeed = + min(turnKp*turnError,0.2 *MAX_SPEED);
  }else{
    leftSpeed  = + min(turnKp*turnError, 0.2 * MAX_SPEED);
    rightSpeed = - min(turnKp*turnError,0.2 * MAX_SPEED);
  }
  } else{
    if(s == 'L'){
      leftSpeed  = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
    }else{
      leftSpeed  = 0.5 * MAX_SPEED;
      rightSpeed = - 0.5 * MAX_SPEED;
    }
  }
  
  //std::cout <<"error: " << abs(turnError) <<std::endl;
  if(abs(turnError) < 1 * (M_PI/180)){
    leftSpeed = 0.0;
    rightSpeed = 0.0;
    prevTime = robot->getTime();
    setMotors();
    updatePosition(s);

    printCurrentPosition();
    return 1;
  }
  setMotors();
  return 0;
}
void PhaseAFunctions::updateWalls(){
 if(currentDirection == 'N'){
      // std::cout <<"NORTH"<<std::endl;
       // std::cout<<"In front: "<<frontWall<<std::endl;
    // std::cout<<"Left: " <<leftWall<<std::endl;
     // std::cout<<"Right: " <<rightWall<<std::endl;
     // std::cout <<"BACK: " <<backWall <<std::endl;
   if(frontWall == 'Y'){
     northWall = true;
   }else{
     northWall = false;
   }
   if(backWall == 'Y'){
     southWall = true;
   }else{
     southWall = false;
   }
   if(rightWall == 'Y'){
     eastWall = true;
   }else{
     eastWall = false;
   }
   if(leftWall == 'Y'){
     rightWall = true;
   }else{
     rightWall = false;
   }
 }else if(currentDirection == 'S'){
      // std::cout <<"SOUTH"<<std::endl;
       // std::cout<<"In front: "<<frontWall<<std::endl;
    // std::cout<<"Left: " <<leftWall<<std::endl;
     // std::cout<<"Right: " <<rightWall<<std::endl;
     // std::cout <<"BACK: " <<backWall <<std::endl;
    if(frontWall == 'Y'){
     southWall = true;
   }else{
     southWall = false;
   }
   if(backWall == 'Y'){
     northWall = true;
   }else{
     northWall = false;
   }
   if(rightWall == 'Y'){
     westWall = true;
   }else{
     westWall = false;
   }
   if(leftWall == 'Y'){
     eastWall = true;
   }else{
     eastWall = false;
   }

 }else if(currentDirection == 'E'){
   // std::cout <<"EAST"<<std::endl;
       // std::cout<<"In front: "<<frontWall<<std::endl;
    // std::cout<<"Left: " <<leftWall<<std::endl;
     // std::cout<<"Right: " <<rightWall<<std::endl;
     // std::cout <<"BACK: " <<backWall <<std::endl;
   if(frontWall == 'Y'){
     eastWall = true;
   }else{
     eastWall = false;
   }
   if(backWall == 'Y'){
     westWall = true;
   }else{
     westWall = false;
   }
   if(rightWall == 'Y'){
     southWall = true;
   }else{
     southWall = false;
   }
   if(leftWall == 'Y'){
     northWall = true;
   }else{
     northWall = false;
   }
 }else if(currentDirection == 'W'){
   if(frontWall == 'Y'){
     westWall = true;
   }else{
     westWall = false;
   }
   if(backWall == 'Y'){
     eastWall = true;
   }else{
     eastWall = false;
   }
   if(rightWall == 'Y'){
     northWall = true;
   }else{
     northWall = false;
   }
   if(leftWall == 'Y'){
     southWall = true;
   }else{
     southWall = false;
   }
 }
 

}
void PhaseAFunctions::updatePosition(char movement){
 //Updating the Row
 stepNumber = stepNumber + 1;
 if(currentDirection == 'N'){
   expectedYaw = -90.00;
   if(movement == 'F'){
     currentRow = currentRow - 1;
   }else if(movement == 'L'){
     currentDirection = 'W';
     expectedYaw = 90.00;
   }else if(movement == 'R'){
     currentDirection = 'E';
     expectedYaw = -90.00;
   }
 }else if(currentDirection == 'S'){
   if(movement == 'F'){
     currentRow = currentRow + 1;
   }else if(movement == 'L'){
     currentDirection = 'E';
     expectedYaw = -90.00;
   }else if(movement == 'R'){
     currentDirection = 'W';
     expectedYaw = 90.00;
   }
 }else if(currentDirection == 'E'){
   if(movement == 'F'){
     currentCol = currentCol + 1;
   }else if(movement == 'L'){
     currentDirection = 'N';
     expectedYaw = 0.00;
   }else if(movement == 'R'){
     currentDirection = 'S';
     expectedYaw = -180.00;
   }
 }else if(currentDirection == 'W'){
   if(movement == 'F'){
     currentCol = currentCol - 1;
   }else if(movement == 'L'){
     currentDirection = 'S';
     expectedYaw = -180.00;
   }else if(movement == 'R'){
     currentDirection = 'N';
     expectedYaw = 0.00;
   }
 }
    
}

void PhaseAFunctions::printCurrentPosition(){
  getDistanceSensorReadings();
  updateWalls();
  string s = to_string(stepNumber) + ","+ to_string(currentRow) + "," + to_string(currentCol) + ","  + currentDirection + "," + leftWall + "," + frontWall + "," + rightWall + "\n";
  motionExecution.push_back(s);
  // std::cout<<"[z5162972_MTRN4110_PhaseA] "<< "Step: "<< std::setw(3) << setfill('0')<<stepNumber<< ", Row: "<< this->currentRow << ", Column: " << this->currentCol << " Heading: " << this->currentDirection<<", Left Wall: "<<leftWall<< ", Front Wall: "<<frontWall<< ", Right Wall: "<< rightWall<< ", Back Wall: " <<backWall<<std::endl;
   std::cout<<"[z5162972_MTRN4110_PhaseA] "<< "Step: "<< std::setw(3) << setfill('0')<<stepNumber<< ", Row: "<< this->currentRow << ", Column: " << this->currentCol << " Heading: " << this->currentDirection<<", North Wall: "<<northWall<< ", South Wall: "<<southWall<< ", East Wall: "<< eastWall<< ", West Wall: " <<westWall<<std::endl;
  // std::cout<<" "<<std::endl;
  
   this->addWalls();
}

void PhaseAFunctions::getDistanceSensorReadings(){
  for (int i = 0; i < 4 ; i++){
      dsValues[i] = ds[i]->getValue();
      // /std::cout<<"reading: " <<dsValues[i]<<std::endl;
    }
  
  
  if(dsValues[0] < 900){
    frontWall = 'Y';
  }else{
    frontWall = 'N';
  }
  if(dsValues[1] < 900){
    leftWall = 'Y';
  }else{
    leftWall = 'N';
  }
  if(dsValues[2] < 900){
    rightWall = 'Y';
  }else{
    rightWall = 'N';
  }
  if(dsValues[3] < 900){
    backWall = 'Y';
  }else{
    backWall = 'N';
  }
    // std::cout<<"In front: "<<dsValues[0]<<std::endl;
    // std::cout<<"Left: " <<dsValues[1]<<std::endl;
     // std::cout<<"Right: " <<dsValues[2]<<std::endl;
     // std::cout <<"BACK: " <<dsValues[3] <<std::endl;
    
    
    // //RIGHT WALL
    // if(dsValues[2] < 950){
    
      // leftSpeed  = 0.5 * MAX_SPEED + pid(dsValues[2] - 750);
      // rightSpeed = 0.5 * MAX_SPEED - pid(dsValues[2] - 750);
      
    // if(leftSpeed < -MAX_SPEED){
      // leftSpeed = -0.5 *MAX_SPEED;
    // }
    // if (rightSpeed < -MAX_SPEED){
      // rightSpeed = -0.5 * MAX_SPEED;
    // }
    // if (leftSpeed >MAX_SPEED){
      // leftSpeed = 0.5 *MAX_SPEED;
    // }
    // if (rightSpeed > MAX_SPEED){
      // rightSpeed = 0.5* MAX_SPEED;
    // }
     // if(dsValues[1] < 950){
    // leftSpeed  = 0.5 * MAX_SPEED - pid(dsValues[1] - 750);
    // rightSpeed = 0.5 * MAX_SPEED + pid(dsValues[1] - 750);
      
    // if(leftSpeed < -MAX_SPEED){
      // leftSpeed = -0.5* MAX_SPEED;
    // }
    // if (rightSpeed < -MAX_SPEED){
      // rightSpeed = -0.5*MAX_SPEED;
    // }
    // if (leftSpeed >MAX_SPEED){
      // leftSpeed =0.5* MAX_SPEED;
    // }
    // if (rightSpeed > MAX_SPEED){
      // rightSpeed = 0.5*MAX_SPEED;
    // }
     
   // }else{
      // leftSpeed = 0.5 * MAX_SPEED;
      // rightSpeed = leftSpeed;
    // }
}
void PhaseAFunctions::generateCSV(){
  outputFile.open ("../../MotionExecution.csv");
  outputFile << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
  for (auto i = motionExecution.cbegin(); i != motionExecution.cend(); ++i)
        outputFile << *i;
  
  outputFile.close();

}

double PhaseAFunctions::pid(double error){
  //std::cout <<"error: " << error <<std::endl;
  prop = error;
  intg = error + intg;
  diff = error - prevError;
  prevError = error;
  return (kp*prop) + (ki*intg) + (kd*diff);
 
}

void PhaseAFunctions::initialiseFloodFill(int start_x,int start_y){
     for(int k = 0; k < 5; k++){
      for (int j = 0; j < 9; j++){
          floodfill[j][k] = 100;
      }
      std::cout <<"" <<std::endl;
    }
    

    
     floodfill[start_x][start_y] = 0;
}

void PhaseAFunctions::printFloodFill(){
    for(int k = 0; k < 5; k++){
      for (int j = 0; j < 9; j++){
          if(floodfill[j][k] == 100){
          std::cout <<"-" <<"    "; 
          }else{
            std::cout <<floodfill[j][k] <<"    "; 
          }
      }
       std::cout <<"" <<std::endl;
    }
    
}

void PhaseAFunctions::BFS(int start_x, int start_y,int goal_x,int goal_y,int currentExploredValue)
{
    
    // std::cout<<"BFS"<<" current: "<< currentExploredValue<<std::endl;
    int j = start_x;
    int k = start_y;
    if(floodfill[j][k] == 100){
      floodfill[j][k] = currentExploredValue;
      // std::cout<<"j: "<<j<<" k: "<<k<<std::endl;
    }
    // printFloodFill();
    if((start_x == goal_x && start_y == goal_y) ||this->foundPath ==1){
      std::cout<<"GOOOOOOOOAAAAAAAAALLLLL"<<std::endl;
      this->foundPath = 1;
      return;
    }
    
    //NORTH
    if(k >= 0){
  
      if(k!=0){
      // std::cout<<"NORTH"<<std::endl;
        if(northWall != 1 && currentExploredValue + 1 < floodfill[j][k-1]){
          if(currentDirection == 'N'){
             // std::cout<<"NORTH: "<<k-1<<std::endl;
             forward();
             
             BFS(j,k-1,goal_x,goal_y,currentExploredValue+1);
             if(foundPath == 1) return;
             right();
             right();
             forward();
             right();
             right();
           }else if(currentDirection == 'E'){
             left();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             right();

           }else if(currentDirection == 'W'){
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();

           }else{
             right();
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();
             left();
           }
        
        
        }
      }
    }
    //SOUTH
    if(k <= 4){
      
      if(k!=4){
        if(southWall != 1 && currentExploredValue +1< floodfill[j][k+1] ){
          
           if(currentDirection == 'S'){
             // std::cout<<"SOUTH: "<<k+1<<std::endl;
             forward();
             
             BFS(j,k+1,goal_x,goal_y,currentExploredValue+1);
             if(foundPath == 1) return;
             right();
             right();
             forward();
             right();
             right();
           }else if(currentDirection == 'W'){
             left();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             right();

           }else if(currentDirection == 'E'){
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();
           }else{
             right();
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();
             left();
           }
        
        }
      }
    }
     //EAST
    if(j <= 9){
      // std::cout<<"EAST"<<std::endl;
      if(j!=9){
        if(eastWall != 1 && currentExploredValue +1 < floodfill[j+1][k]){
          
          if(currentDirection == 'E'){
             // std::cout<<"EAST: "<<j+1<<std::endl;
             forward();
             BFS(j+1,k,goal_x,goal_y,currentExploredValue+1);
             if(foundPath == 1) return;
             right();
             right();
             forward();
             right();
             right();
           }else if(currentDirection == 'S'){
             left();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             right();
           }else if(currentDirection == 'N'){
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();
           }else{
             right();
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();
             left();
           }
  
        
        }
      }
    }
    
    //WEST
    if( j >= 0 ){
      // std::cout<<"WEST"<<std::endl;
      if(j!= 0){
        if(westWall != 1 && currentExploredValue+1 < floodfill[j-1][k]){
          if(currentDirection == 'W'){
            // std::cout<<"WEST: "<<j+1<<std::endl;
             forward();
             BFS(j-1,k,goal_x,goal_y,currentExploredValue+1);
             if(foundPath == 1) return;
             right();
             right();
             forward();
             right();
             right();
           }else if(currentDirection == 'N'){
             left();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             right();
           }else if(currentDirection == 'S'){
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();
           }else{
             right();
             right();
             BFS(j,k,goal_x,goal_y,currentExploredValue);
             if(foundPath == 1) return;
             left();
             left();
           }
        
        }
      }
      
      
    }
    
    
    return;
}

void PhaseAFunctions::forward(){
  bool move_complete = 0;
  haltRobot(leftMotor, rightMotor);
  getImu();
  getDistanceSensorReadings();
  int encoderOffsetLeft = leftMotorSensor->getValue();
  int encoderOffsetRight = rightMotorSensor->getValue();
  // std::cout<< "encoder Left: " <<encoderOffsetLeft << " Encoder Right: " << encoderOffsetRight <<std::endl;
  startPosLeft = encoderOffsetLeft;
  startPosRight = encoderOffsetRight;
  while (robot->step(TIME_STEP) != -1){
     // move_complete = move_Forward();
     move_complete = moveForward();
     // std::cout<<"forward"<<std::endl;
     if(move_complete == 1){
       // updatePosition('F');
       // printCurrentPosition();
       break;
     }
   }
  return;
}
void PhaseAFunctions::left(){
  bool move_complete = 0;
  getImu();
  getDistanceSensorReadings();
  haltRobot(leftMotor, rightMotor);
  int encoderOffsetLeft = leftMotorSensor->getValue();
  int encoderOffsetRight = rightMotorSensor->getValue();

  // std::cout<< encoderOffsetLeft << " " << encoderOffsetRight <<std::endl;

  while (robot->step(TIME_STEP) != -1){
     // move_complete = turnLeft(encoderOffsetLeft,encoderOffsetRight);
     move_complete = turn('L');
     if(move_complete == 1){
       // updatePosition('L');
       // printCurrentPosition();
       break;
     }
   }
  return;
}
void PhaseAFunctions::right(){
  bool move_complete = 0;
  haltRobot(leftMotor, rightMotor);
  // getImu();
  // getDistanceSensorReadings();
  int encoderOffsetLeft = leftMotorSensor->getValue();
  int encoderOffsetRight = rightMotorSensor->getValue();
  // std::cout<< encoderOffsetLeft << " " << encoderOffsetRight <<std::endl;
  while (robot->step(TIME_STEP) != -1){
     // move_complete = turnRight(encoderOffsetLeft,encoderOffsetRight);
     move_complete = turn('R');
     if(move_complete == 1){
       // updatePosition('R');
       // printCurrentPosition();
       break;
     }
   }
  return;
}

void PhaseAFunctions::addWalls(){

  int row = this->currentRow * 2;
  int col = this->currentCol * 2;
  if (this->northWall == 1){
    // std::cout<<"NORTH"<<std::endl;
    this->map[row][col+1] = 1;
    // std::cout<<(row) <<" "<<(col+1) <<std::endl;
  }
  if (this->southWall == 1){
  // std::cout<<"SOUTH"<<std::endl;
    this->map[row+2][col+1] = 1;
  }
  if (this->eastWall == 1){
  // std::cout<<"EAST"<<std::endl;
    this->map[row+1][col+2] = 1;
  }
  if (this->westWall == 1){
    // std::cout<<"WEST"<<std::endl;
    this->map[row+1][col] = 1;
  }
  
  // for (int j = 0; j < 11;j++){
    // for(int k = 0; k < 19; k++){
    // std::cout<<map[j][k]<<" ";
    // }
    // std::cout<<" "<<std::endl;
  // }

    string ret;
    for(int height = 0; height < 11;height++){
        for(int width = 0; width <19; width++){
            if(height%2){
                if(width%2 == 0){
    // #                 print(width)
                    if(map[height][width] == 1){
                         // print("|", end="")
                        ret = ret + "|";
                    }else{
                         // print(" ", end="")
                        ret = ret + " ";
                    }
                }else{
                    if(map[height][width] == 2){
                         // print(" ^ ", end="")
                        ret = ret + " ^ ";
                    }else if(map[height][width] == 3){
                        // print(" v ", end="")
                        ret = ret + " v ";
                    }else if(map[height][width] == 4){
                         // print(" > ", end="")
                        ret = ret + " > ";
                    }else if(map[height][width] == 5){
                         // print(" < ", end="")
                        ret = ret + " <";
                    }else if(map[height][width] == 6){
                         // print(" x ", end="")
                        ret = ret + " x ";
                    }else{
                         // print("   ", end="")
                        ret = ret + "   ";
                    }
                }
           } else{

                if(width%2==0){
                     // print(" ", end="")
                    ret = ret + " ";
                }else{
                    if(map[height][width] == 1){
                         // print("...", end="")
                        ret = ret + "...";
                    }else{
                         // print("   ", end="")
                        ret = ret + "   ";
                    }
              }
        }
         // print("\n")
        
      }
      ret = ret + "\n";
    }
    std::cout<< ret<<std::endl;
    
    return;
}

