//
//  PhaseBFunctions.cpp
//  
//
//  Created by James Davies on 17/8/21.
//

#include "PhaseBFunctions.hpp"

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
void printMapFromCoords(char map[ROWS][CHARS_PER_LINE], vector<vector<int> > coord, vector<int> src)
{
    int colCount = 0;
    bool printRest = false;
    int numToPrint;
    for (int i = 0; i < ROWS; i++) {
        cout << CONSOLE_PREFIX;
        for (int j = 0; j < CHARS_PER_LINE; j++) {
            if ((i + 1) % 2 == 0) {
                // is in a number row, check if column requires a number
                if (j == 2) colCount = 0; // begin checks
                else if (j > 2) colCount++;
                
                // check if position is the same as start position, if so print heading
                if (i == src[0] && j == src[1]) {
                    cout << map[i][j];
                    continue;
                }
                
                if (printRest) { // print rest of the number (units)
                    cout << numToPrint % 10;
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
                        } else {
                            cout << numToPrint; // print units (single digit num)
                        }
                    } else {
                        cout << " "; // print spaces
                    }
                } else {
                    cout << map[i][j]; // speed through printing
                }
            } else {
                cout << map[i][j]; // is in a wall row, speed through outputting
            }
        }
        cout << endl;
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
