//
//  PhaseBFunctions.hpp
//  
//
//  Created by James Davies on 17/8/21.
//

#ifndef PhaseBFunctions_hpp
#define PhaseBFunctions_hpp

#include <iostream>

const char END_POINT = 'x';
const int CHARS_PER_LINE = 37;
const int ROWS = 11;
const int SPACE_BETWEEN_CENTERS = 4;
const int NUM_ROWS = 5; // number of coordinate rows
const int NUM_COLS = 9; // number of coordinate columns

enum directions
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};

// function declarations
bool isStartPoint(char pt);
vector<int> positionToCoord(int pos);
void printMap(int map[NUM_ROWS][NUM_COLS]);
void printMapFromCoords(char map[ROWS][CHARS_PER_LINE], vector<vector<int> > coord,
                        vector<int> src);
int isInVector(int x, int y, vector<vector<int> > coord);
int pathToString(vector<char> &pathString, vector<vector<int> > path, int orientation,
                 vector<int> dest, vector<int> src);
void left(vector<char> &pathString, int &charCount);
void flip(vector<char> &pathString, int &charCount);
void right(vector<char> &pathString, int &charCount);
void straight(vector<char> &pathString, int &charCount);
#endif /* PhaseBFunctions_hpp */
