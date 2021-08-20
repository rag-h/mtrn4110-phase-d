//
//  PhaseAFunctions.cpp
//  
//
//  Created by James Davies on 15/8/21.
//

#include "PhaseAFunctions.hpp"

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define PI 3.14193
#define MOTION_PLAN_FILE_NAME "../../MotionPlan.txt"
#define CELL_LENGTH_RAD 8.25
#define CELL_ROTATION 2.232
#define TOL1 0.4
#define TOL2 0.009
const string CONSOLE_PREFIX = "[Group16_MTRN4110_PhaseD] ";

int checkStepComplete(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing, double currentRange, int forward, int turning){
    
    if(abs(currentLW - nextLW) > TOL2){
        return 0;
    } else if (abs(currentRW - nextRW) > TOL2){
        return 0;
    }
    return 1;
    
}

int checkStepNearlyDone(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing, int forward, int turning){
    
    if(abs(currentLW - nextLW) > TOL1){
        return 0;
    } else if (abs(currentRW - nextRW) > TOL1){
        return 0;
    }
    return 1;
    
}

void checkWalls(char* walls, int dsFValue, int dsRValue, int dsLValue){
    
    char lWall = 'N';
    char rWall = 'N';
    char fWall = 'N';
    
    if(dsFValue > 600){
        fWall = 'Y';
    } else {
        fWall = 'N';
    }
    walls[0] = fWall;
    
    if(dsRValue > 600){
        rWall = 'Y';
    } else {
        rWall = 'N';
    }
    walls[1] = rWall;
    
    if(dsLValue > 600){
        lWall = 'Y';
    } else {
        lWall = 'N';
    }
    walls[2] = lWall;
    
}

void printStep(int step, int row, int col, char heading, char lWall, char fWall, char rWall){
    std::cout << CONSOLE_PREFIX << "Step: ";
    if (step < 13){
        std::cout << "00";
    }
    if (step >= 13 && step < 103){
        std::cout << "0";
    }
    std::cout << (step-3)
    << ", Row: " << row
    << ", Column: " << col
    << ", Heading: " << heading
    << ", Left Wall: " << lWall
    << ", Front Wall: " << fWall
    << ", Right Wall: " << rWall
    << std::endl;
    
}
