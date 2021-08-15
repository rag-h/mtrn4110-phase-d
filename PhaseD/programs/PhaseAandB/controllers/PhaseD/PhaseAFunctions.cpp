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
#define MOTION_PLAN_FILE_NAME "../../PathPlan.txt"
#define MAP_FILE_NAME "../../../../MapBuilt.txt"
#define CELL_LENGTH_RAD 8.25
//#define CELL_ROTATION 2.2227
#define CELL_ROTATION 2.232

#define TOL1 0.4
#define TOL2 0.009

int checkStepComplete(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing, double currentRange, int forward, int turning){
    
    // std::cout << "currentLW: " << currentLW << std::endl;
    // std::cout << "nextLW: " << nextLW << std::endl;
    // std::cout << "currentRW: " << currentRW << std::endl;
    // std::cout << "nextRW: " << nextRW << std::endl;
    // std::cout << "currentRange: " << currentRange << std::endl;
    // std::cout << "compassBearing: " << compassBearing << std::endl;
    // std::cout << "LW Diff: " << abs(currentLW - nextLW) << std::endl;
    // std::cout << "RW Diff: " << abs(currentRW - nextRW) << std::endl;
    
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
    std::cout << "[z5162711_MTRN4110_PhaseA] Step: ";
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
