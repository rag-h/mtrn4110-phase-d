//
//  PhaseAFunctions.hpp
//  
//
//  Created by James Davies on 15/8/21.
//

#ifndef PhaseAFunctions_hpp
#define PhaseAFunctions_hpp

#include <iostream>


// Function Definitions - Phase A
int checkStepComplete(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing, double currentRange, int forward, int turning);
int checkStepNearlyDone(double currentLW, double currentRW, double nextLW, double nextRW, double compassBearing,int forward, int turning);
void checkWalls(char* walls, int dsFValue, int dsRValue, int dsLValue);
void printStep(int step, int row, int col, char heading, char lWall, char fWall, char rWall);


#endif /* PhaseAFunctions_hpp */
