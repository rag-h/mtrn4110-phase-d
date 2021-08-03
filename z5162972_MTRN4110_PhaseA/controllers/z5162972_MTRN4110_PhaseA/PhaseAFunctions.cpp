#include "PhaseAFunctions.hpp"

PhaseAFunctions::PhaseAFunctions(Robot *_robot){
  this->robot = _robot;
  
  this->leftMotor = _robot->getMotor("left wheel motor");
  this->rightMotor = _robot->getMotor("right wheel motor");
  this->leftMotor->setPosition(INFINITY);
  this->rightMotor->setPosition(INFINITY);
  this->leftMotor->setVelocity(0.0);
  this->rightMotor->setVelocity(0.0);
  this->imu = robot->getInertialUnit("imu");
  this->imu->enable(TIME_STEP);  
  
  for (int i = 0; i < 3; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);    
    std::cout<< " " <<std::endl;
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
  
  if(robot->getTime() >= (prevTime + CELL_LENGTH/(WHEEL_RADIUS*0.5*MAX_SPEED)) || dsValues[0] < 750)  {
    leftSpeed = 0.0;
    rightSpeed = 0.0;
    prevTime = robot->getTime();
    setMotors();
    updatePosition('F');
    printCurrentPosition();
    return 1;
  }
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
  string s = to_string(stepNumber) + ","+ to_string(currentRow) + "," + to_string(currentCol) + ","  + currentDirection + "," + leftWall + "," + frontWall + "," + rightWall + "\n";
  motionExecution.push_back(s);
  std::cout<<"[z5162972_MTRN4110_PhaseA] "<< "Step: "<< std::setw(3) << setfill('0')<<stepNumber<< ", Row: "<< this->currentRow << ", Column: " << this->currentCol << " Heading: " << this->currentDirection<<", Left Wall: "<<leftWall<< ", Front Wall: "<<frontWall<< ", Right Wall: "<< rightWall<<std::endl;

}

void PhaseAFunctions::getDistanceSensorReadings(){
  for (int i = 0; i < 3 ; i++){
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
    // std::cout<<"In front: "<<dsValues[0]<<std::endl;
    // std::cout<<"Left: " <<dsValues[1]<<std::endl;
    // std::cout <<"Right: " <<dsValues[2] <<std::endl;
    
    
    // //RIGHT WALL
    if(dsValues[2] < 950){
      leftSpeed  = 0.5 * MAX_SPEED + pid(dsValues[2] - 750);
      rightSpeed = 0.5 * MAX_SPEED - pid(dsValues[2] - 750);
      
    if(leftSpeed < -MAX_SPEED){
      leftSpeed = -MAX_SPEED;
    }
    if (rightSpeed < -MAX_SPEED){
      rightSpeed = -MAX_SPEED;
    }
    if (leftSpeed >MAX_SPEED){
      leftSpeed = MAX_SPEED;
    }
    if (rightSpeed > MAX_SPEED){
      rightSpeed = MAX_SPEED;
    }

    // }else if(dsValues[1] < 950){
      // leftSpeed  = 0.5 * MAX_SPEED - pid(dsValues[1] - 750);
      // rightSpeed = 0.5 * MAX_SPEED + pid(dsValues[1] - 750);
      
    // if(leftSpeed < -MAX_SPEED){
      // leftSpeed = -MAX_SPEED;
    // }
    // if (rightSpeed < -MAX_SPEED){
      // rightSpeed = -MAX_SPEED;
    // }
    // if (leftSpeed >MAX_SPEED){
      // leftSpeed = MAX_SPEED;
    // }
    // if (rightSpeed > MAX_SPEED){
      // rightSpeed = MAX_SPEED;
    // }
            
    }else{
      leftSpeed = 0.5 * MAX_SPEED;
      rightSpeed = leftSpeed;
    }
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