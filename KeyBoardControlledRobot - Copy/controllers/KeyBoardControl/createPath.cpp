// File:          z5162966_MTRN4110_PhaseB.cpp
// Date:          22/06/2021
// Description:   Controller of E-puck for Phase B - Path Planning
// Author:        Luke Jackson
// Platform:      Windows
// Modifications:

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
#include "directions.h"
#include "constants.h"

// All the webots classes are defined in the "webots" namespace
using namespace std;

// function declarations
bool isStartPoint(char pt);
vector<int> positionToCoord(int pos);
void printMap(int map[NUM_ROWS][NUM_COLS]);
void printMapFromCoords(char map[ROWS][CHARS_PER_LINE], vector<vector<int> > coord, 
    vector<int> src, ostream &outputHandle);
int isInVector(int x, int y, vector<vector<int> > coord);
int pathToString(vector<char> &pathString, vector<vector<int> > path, int orientation,
    vector<int> dest, vector<int> src);
void left(vector<char> &pathString, int &charCount);
void flip(vector<char> &pathString, int &charCount);
void right(vector<char> &pathString, int &charCount);
void straight(vector<char> &pathString, int &charCount);
vector<string> createFirstPath();
vector<string> createNewPath(vector<int> currPos, directions currHeading, vector<int> targetPos);
vector<int> coordToGlobalFrame(vector<int> coord);
int coordToPoint(vector<int> coord);

using namespace std;

vector<string> createFirstPath() {
    vector<int> startingPoint;
    vector<int> endingPoint;
    startingPoint.push_back(0);
    startingPoint.push_back(0);
    endingPoint.push_back(4);
    endingPoint.push_back(4);
    cout << "from: (" << startingPoint[0] << "," << startingPoint[1] << ") = " << coordToPoint(startingPoint) << endl;
    cout << "to: (" << endingPoint[0] << "," << endingPoint[1] << ") = " << coordToPoint(endingPoint) << endl;
    createNewPath(startingPoint, SOUTH, endingPoint);
}

vector<string> createNewPath(vector<int> currPos, directions currHeading, vector<int> targetPos) {
    // Read in the map text file
    ifstream mapFileHandle(MAP_FILE_NAME); // open the file
    string lineTemp;
    char map[ROWS][CHARS_PER_LINE]; // 11 rows of 38 columns

    for (int i = 0; i < ROWS; i++) {
        getline(mapFileHandle, lineTemp);
        for (int j = 0; j < CHARS_PER_LINE; j++) {
            map[i][j] = lineTemp[j];
        }
    }
    mapFileHandle.close();
    
    // print out the map text file
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < CHARS_PER_LINE; j++) {
            cout << map[i][j];
        }
        cout << endl;
    }
    vector<int> startPosGlobal = coordToGlobalFrame(currPos);
    // begin path planning
    Graph g(NUM_ROWS, NUM_COLS);
    g.createGraph(currPos, targetPos, map);
    vector<vector<int> > newAdjList = g.getAdjList();
    /*
    cout << "printing the adjacency list" << endl;
    int i = 0;
    for (vector<int> outer: newAdjList) {
        cout << "[" << i << "] -> ";
        for (int inner: outer) {
            cout << inner << " ";
        }
        cout << endl;
        i++;
    }
    cout << "done printing the adjacency list" << endl;
    */
   
   int pathCounter = 1;
   vector<vector<vector<int> > > allPaths = getPathCoords(newAdjList, coordToPoint(currPos), coordToPoint(targetPos), NUM_ROWS, NUM_COLS);

    // Find the path with fewest turns
    for (vector<vector<int> > path : allPaths) {
        cout << "Path - " << pathCounter << ":" << endl;
        reverse(path.begin(), path.end());
    }
    cout << pathCounter - 1 << " shortest paths found!" << endl;
    // Find the minimum number of turns:
    // loop through all paths, create String movements for all, shortest string
    // is the shortest path
    cout << "Finding shortest path with least turns..." << endl;
    vector<char> shortestPathString;
    int shortestPathLength = NUM_ROWS * NUM_COLS * 3;
    int shortestPathIndex = 0;
    int counter = 0;

    for (vector<vector<int> > path : allPaths) {
        vector<char> pathString;
        int length = pathToString(pathString, path, currHeading, targetPos, currPos);
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

    // printing the path
    cout << "Shortest path found!" << endl;
    cout << "Path Plan (" << shortestPathLength << " steps): ";
    for (int step: shortestPathString) {
        cout << (char)step;
    }
    cout << endl;

}

// converts a coord in a 9 x 5 grid to coord from the text file
vector<int> coordToGlobalFrame(vector<int> coord) {    
    return vector<int>(coord[0] * 2 + 1, coord[1] * 4 + 2);
}

// returns numerical positon given coord in [row,col]  form
int coordToPoint(vector<int> coord) {
    return coord[0] * NUM_COLS + coord[1];
}


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
void printMapFromCoords(char map[ROWS][CHARS_PER_LINE], vector<vector<int> > coord, vector<int> src, ostream &outputHandle)
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
int isInVector(int x, int y, vector<vector<int> > coord)
{
    for (int i = 0; i < (int)coord.size(); i++) {
        if (coord[i][0] == x && coord[i][1] == y) return i;
    }
    return -1;
}

int pathToString(vector<char> &pathString, vector<vector<int> > path, int orientation, vector<int> dest, vector<int> src)
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