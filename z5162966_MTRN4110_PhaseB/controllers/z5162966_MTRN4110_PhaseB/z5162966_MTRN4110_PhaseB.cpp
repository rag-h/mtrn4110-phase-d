// File:          z5162966_MTRN4110_PhaseB.cpp
// Date:          22/06/2021
// Description:   Controller of E-puck for Phase B - Path Planning
// Author:        Luke Jackson
// Platform:      Windows
// Modifications:

#include <webots/Robot.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <cmath>
#include <deque>
#include <vector>
#include <queue>
#include <algorithm>

#include "path.cpp"
#include "graph.cpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

const string MAP_FILE_NAME = "../../Map.txt";
const string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const string OUTPUT_FILE_NAME = "../../Output.txt";
const string CONSOLE_PREFIX = "[z5162966_MTRN4110_PhaseB] ";


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
void printMapFromCoords(char map[ROWS][CHARS_PER_LINE], vector<vector<int>> coord, 
    vector<int> src, ostream &outputHandle);
int isInVector(int x, int y, vector<vector<int>> coord);
int pathToString(vector<char> &pathString, vector<vector<int>> path, int orientation,
    vector<int> dest, vector<int> src);
void left(vector<char> &pathString, int &charCount);
void flip(vector<char> &pathString, int &charCount);
void right(vector<char> &pathString, int &charCount);
void straight(vector<char> &pathString, int &charCount);



int main(int argc, char **argv)
{
    // create the Robot instance.
    Robot *robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // open text file for output of command line to
    ofstream outputHandle(OUTPUT_FILE_NAME);
    
    // Read in the map text file
    ifstream mapFileHandle(MAP_FILE_NAME); // open the file
    string lineTemp;
    char map[ROWS][CHARS_PER_LINE]; // 11 rows of 38 columns
    cout << CONSOLE_PREFIX << "Reading in map from ../../Map.txt..." << endl;
    outputHandle << CONSOLE_PREFIX << "Reading in map from ../../Map.txt..." << endl;
    // get the characters one at a time
    for (int i = 0; i < ROWS; i++) {
        getline(mapFileHandle, lineTemp);
        for (int j = 0; j < CHARS_PER_LINE; j++) {
            map[i][j] = lineTemp[j];
        }
    }
    // print out the map text file
    for (int i = 0; i < ROWS; i++) {
        cout << CONSOLE_PREFIX;
        outputHandle << CONSOLE_PREFIX;
        for (int j = 0; j < CHARS_PER_LINE; j++) {
            cout << map[i][j];
            outputHandle << map[i][j];
        }
        cout << endl;
        outputHandle << endl;
    }
    cout << CONSOLE_PREFIX << "Map read in!" << endl;
    outputHandle << CONSOLE_PREFIX << "Map read in!" << endl;
    mapFileHandle.close();

    // find coord of start and end positon
    int orientation = SOUTH;
    int rowCoord = 0;
    int startLoc = 0;
    int endLoc = 0;
    vector<int> startPosGlobal;
    for (int row = 1; row < ROWS; row += 2) {
        int colCoord = 0;
        for (int col = 2; col < CHARS_PER_LINE; col += SPACE_BETWEEN_CENTERS) {
            int currPos = (rowCoord * NUM_COLS) + colCoord;
            // finding if start or end point
            if (isStartPoint(map[row][col])) {
                if (map[row][col] == 'v') orientation = SOUTH;
                else if (map[row][col] == '>') orientation = EAST;
                else if (map[row][col] == '<') orientation = WEST;
                else if (map[row][col] == '^') orientation = NORTH;
                startPosGlobal.push_back(row);
                startPosGlobal.push_back(col);
                startLoc = currPos;
            }
            if (map[row][col] == 'x') endLoc = currPos;
            colCoord++;
        }
        rowCoord++;
    }
    // convert start and end postion to coordainates (row, column)
    vector<int> startPos = positionToCoord(startLoc);
    vector<int> endPos = positionToCoord(endLoc);
    // flood fill
    // initialize array and fill with high number
    int mapArray[NUM_ROWS][NUM_COLS];
    for (int i = 0; i < NUM_ROWS; i++) for (int j = 0; j < NUM_COLS; j++) mapArray[i][j] = NUM_ROWS * NUM_COLS;
    // begin flood fill algorithm
    mapArray[endPos[0]][endPos[1]] = 0;
    int currExploredValue = 0;
    int mazeValueChanged = 1;
    // loop through all values
    while (mazeValueChanged != 0) {
        mazeValueChanged = 0;
        for (int i = 0; i < NUM_ROWS; i++) {
            for (int j = 0; j < NUM_COLS; j++) {
                if (mapArray[i][j] == currExploredValue) {
                    char walls[4]; // North, East, South, West
                    // find wall characters
                    walls[0] = map[i * 2][j * SPACE_BETWEEN_CENTERS + 2];
                    walls[1] = map[i * 2 + 1][j * SPACE_BETWEEN_CENTERS + 4];
                    walls[2] = map[i * 2 + 2][j * SPACE_BETWEEN_CENTERS + 2];
                    walls[3] = map[i * 2 + 1][j * SPACE_BETWEEN_CENTERS];
                    for (int k = 0; k < 4; k++) {
                        if (walls[k] == ' ') { // if wall does not exist
                            switch (k) 
                            {
                            case 0:
                                if (mapArray[i - 1][j] == NUM_ROWS * NUM_COLS) {
                                    mapArray[i - 1][j] = mapArray[i][j] + 1;
                                    mazeValueChanged = 1;
                                }
                                break;
                            case 1:
                                if (mapArray[i][j + 1] == NUM_ROWS * NUM_COLS) {
                                    mapArray[i][j + 1] = mapArray[i][j] + 1;
                                    mazeValueChanged = 1;
                                }
                                break;
                            case 2:
                                if (mapArray[i + 1][j] == NUM_ROWS * NUM_COLS) {
                                    mapArray[i + 1][j] = mapArray[i][j] + 1;
                                    mazeValueChanged = 1;
                                }
                                break;
                            case 3:
                                if (mapArray[i][j - 1] == NUM_ROWS * NUM_COLS) {
                                    mapArray[i][j - 1] = mapArray[i][j] + 1;
                                    mazeValueChanged = 1;
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
        currExploredValue++;
    }
    
    // Path Planning alogrithm
    // run through flood fill array constructing a graph 
    // this graph has adjaceny list representation,
    // then BFS this graph to retrieve all shortest paths
    // the convert these paths from array indices to coords
    
    // list of paths contains a list of coordaintes
    // list of coordinates example: [0,0]
    //[[[0,0], [0,1], ..], (new path:) [[1,1], [0,1], ..]
    cout << CONSOLE_PREFIX << "Finding shortest paths..." << endl;
    outputHandle << CONSOLE_PREFIX << "Finding shortest paths..." << endl;
    Graph g(NUM_ROWS, NUM_COLS);
    g.graphFF(NUM_ROWS, NUM_COLS, mapArray, ROWS, CHARS_PER_LINE, map);
    vector<vector<int>> newAdjList = g.getAdjList();
    vector<vector<vector<int>>> allPaths = getPathCoords(newAdjList, startLoc, endLoc, NUM_ROWS, NUM_COLS);

    // Find the path with fewest turns
    int pathCounter = 1;
    for (vector<vector<int>> path : allPaths) {
        cout << CONSOLE_PREFIX << "Path - " << pathCounter << ":" << endl;
        outputHandle << CONSOLE_PREFIX << "Path - " << pathCounter++ << ":" << endl;
        reverse(path.begin(), path.end());
        printMapFromCoords(map, path, startPosGlobal, outputHandle);
    }
    cout << CONSOLE_PREFIX << pathCounter - 1 << " shortest paths found!" << endl;
    outputHandle << CONSOLE_PREFIX << pathCounter - 1 << " shortest paths found!" << endl;
    // Find the minimum number of turns:
    // loop through all paths, create String movements for all, shortest string
    // is the shortest path
    cout << CONSOLE_PREFIX << "Finding shortest path with least turns..." << endl;
    outputHandle << CONSOLE_PREFIX << "Finding shortest path with least turns..." << endl;
    vector<char> shortestPathString;
    int shortestPathLength = NUM_ROWS * NUM_COLS * 3;
    int shortestPathIndex = 0;
    int counter = 0;

    for (vector<vector<int>> path : allPaths) {
        vector<char> pathString;
        int length = pathToString(pathString, path, orientation, endPos, startPos);
        if (length < shortestPathLength) {
            shortestPathString.clear();
            shortestPathLength = length;
            shortestPathIndex = counter;
            // copy string to save
            for (char chars: pathString) {
                shortestPathString.push_back(chars);
            }
        }
        counter++;
    }
    reverse(allPaths[shortestPathIndex].begin(), allPaths[shortestPathIndex].end());
    printMapFromCoords(map, allPaths[shortestPathIndex], startPosGlobal, outputHandle);



    cout << CONSOLE_PREFIX << "Shortest path with least turns found!" << endl;
    cout << CONSOLE_PREFIX << "Path Plan (" << shortestPathLength << " steps): ";
    outputHandle << CONSOLE_PREFIX << "Shortest path with least turns found!" << endl;
    outputHandle << CONSOLE_PREFIX << "Path Plan (" << shortestPathLength << " steps): ";
    for (int step: shortestPathString) {
        cout << (char)step;
        outputHandle << (char)step;
    }
    cout << endl;
    cout << CONSOLE_PREFIX << "Writing path plan to ../../PathPlan.txt..." << endl;
    outputHandle << endl;
    outputHandle << CONSOLE_PREFIX << "Writing path plan to ../../PathPlan.txt..." << endl;

    // write path to file
    ofstream pathPlanHandle;
    pathPlanHandle.open(PATH_PLAN_FILE_NAME); // open the file
    for (char chars: shortestPathString) pathPlanHandle << chars;
    pathPlanHandle.close();
    cout << CONSOLE_PREFIX << "Path plan written to ../../PathPlan.txt!" << endl;
    outputHandle << CONSOLE_PREFIX << "Path plan written to ../../PathPlan.txt!";
    outputHandle.close();
    // - perform simulation steps until Webots is stopping the controller
  
  while (robot->step(timeStep) != -1) {
    cout << "Looping main simulation";

  };
   
  // Enter here exit cleanup code.
  
  delete robot;
  
  return 0;
}


/*
  --------------- HELPER FUNCTIONS ---------------
*/



// check if a coordinate is the satrting point (orientation ignored)
bool isStartPoint(char pt)
{
    if (pt == 'v' || pt == '>' || pt == '<' || pt == '^')
        return true;
    return false;
}

// returns [row,col] coord of a point given a numerical positon
vector<int> positionToCoord(int pos) {
	vector<int> returnCoord;
	if (pos == 0) {
		returnCoord.push_back(0);
		returnCoord.push_back(0);
		return returnCoord;
	}
	
	returnCoord.push_back(floor(pos / NUM_COLS)); // row
	returnCoord.push_back(pos - (NUM_COLS * returnCoord[0])); // col
	return returnCoord;
}

// prints the original map with the path numbers from flood fill added for a
// specified path in the form [[4, 5], [4, 4], [3, 4], ...]
// the path vector starts at the end and returns to the front
void printMapFromCoords(char map[ROWS][CHARS_PER_LINE], vector<vector<int>> coord, vector<int> src, ostream &outputHandle)
{
    int colCount = 0;
    bool printRest = false;
    int numToPrint;
    for (int i = 0; i < ROWS; i++) {
        cout << CONSOLE_PREFIX;
        outputHandle << CONSOLE_PREFIX;
        for (int j = 0; j < CHARS_PER_LINE; j++) {
            if ((i + 1) % 2 == 0) {
                // is in a number row, check if column requires a number
                if (j == 2) colCount = 0; // begin checks
                else if (j > 2) colCount++;

                // check if position is the same as start position, if so print heading
                if (i == src[0] && j == src[1]) {
                    cout << map[i][j];
                    outputHandle << map[i][j];
                    continue;
                }

                if (printRest) { // print rest of the number (units)
                    cout << numToPrint % 10;
                    outputHandle << numToPrint % 10;
                    printRest = false;
                    continue;
                }

                if (colCount % 4 == 0 && j >= 2) {
                    // check if coordinate is the list of coords
                    int x, y;
                    x = (i - 1) / 2;
                    y = (j - 2) / 4;
                    numToPrint = isInVector(x, y, coord);
                    if (numToPrint != -1) {
                        if (numToPrint >= 10) { // check if number takes up multiple columns
                            printRest = true;
                            cout << floor(numToPrint / 10); // print tens value (double digit num)
                            outputHandle << floor(numToPrint / 10);
                        } else {
                            cout << numToPrint; // print units (single digit num)
                            outputHandle << numToPrint;
                        }
                    } else {
                        cout << " "; // print spaces
                        outputHandle << " ";
                    }
                } else {
                    cout << map[i][j]; // speed through printing
                    outputHandle << map[i][j];
                }
            } else {
                cout << map[i][j]; // is in a wall row, speed through outputting
                outputHandle << map[i][j];
            }
        }
        cout << endl;
        outputHandle << endl;
    }
}

// returns the index for the coordinate in the list, -1 if not
int isInVector(int x, int y, vector<vector<int>> coord)
{
    for (int i = 0; i < (int)coord.size(); i++) {
        if (coord[i][0] == x && coord[i][1] == y) return i;
    }
    return -1;
}

int pathToString(vector<char> &pathString, vector<vector<int>> path, int orientation, vector<int> dest, vector<int> src)
{
    // add src and heading
    int charCount = 0;
    pathString.push_back(src[0] + '0'); // stupid int -> char conversion
    pathString.push_back(src[1] + '0');    
    if (orientation == NORTH) pathString.push_back('N');
    else if (orientation == EAST) pathString.push_back('E');
    else if (orientation == SOUTH) pathString.push_back('S');
    else if (orientation == WEST) pathString.push_back('W');

    // add all other movement characters
    for (int i = 0; i < (int)path.size(); i++) {
        vector<int> currCoord = path[i];
        if (currCoord == dest) return charCount;
        vector<int> nextCoord = path[i + 1];
        if (currCoord[0] == (nextCoord[0] + 1) && currCoord[1] == nextCoord[1]) {
            // moving up
            if (orientation == NORTH) straight(pathString, charCount);
            else if (orientation == EAST) left(pathString, charCount);
            else if (orientation == SOUTH) flip(pathString, charCount);
            else if (orientation == WEST) right(pathString, charCount);
            orientation = NORTH;
        }
        if (currCoord[0] == nextCoord[0] && currCoord[1] == (nextCoord[1] - 1)) {
            // moving right
            if (orientation == NORTH) right(pathString, charCount);
            else if (orientation == EAST) straight(pathString, charCount);
            else if (orientation == SOUTH) left(pathString, charCount);
            else if (orientation == WEST) flip(pathString, charCount);
            orientation = EAST;
        }
        if (currCoord[0] == (nextCoord[0] - 1) && currCoord[1] == nextCoord[1]) {
            // moving down
            if (orientation == NORTH) flip(pathString, charCount);
            else if (orientation == EAST) right(pathString, charCount);
            else if (orientation == SOUTH) straight(pathString, charCount);
            else if (orientation == WEST) left(pathString, charCount);
            orientation = SOUTH;
        }
        if (currCoord[0] == nextCoord[0] && currCoord[1] == (nextCoord[1] + 1)) {
            // moving left
            if (orientation == NORTH) left(pathString, charCount);
            else if (orientation == EAST) flip(pathString, charCount);
            else if (orientation == SOUTH) right(pathString, charCount);
            else if (orientation == WEST) straight(pathString, charCount);
            orientation = WEST;
        }
    }
    return charCount;
}

void left(vector<char> &pathString, int &charCount)
{
    pathString.push_back('L');
    pathString.push_back('F');
    charCount += 2;
}

void flip(vector<char> &pathString, int &charCount)
{
    pathString.push_back('L');
    pathString.push_back('L');
    pathString.push_back('F');
    charCount += 3;
}

void right(vector<char> &pathString, int &charCount)
{
    pathString.push_back('R');
    pathString.push_back('F');
    charCount += 2;
}

void straight(vector<char> &pathString, int &charCount)
{
    charCount++;
    pathString.push_back('F');
}