/* implements a graph class structure for creating the graph
    Author: Luke Jackson
    heavily adpated / inspired from these sources:
    https://www.techiedelight.com/graph-implementation-using-stl/ <- most inspiration
    https://www.geeksforgeeks.org/graph-and-its-representations/
    https://www.softwaretestinghelp.com/graph-implementation-cpp/
*/

#include <iostream>
#include <vector>

#include "constants.h"

using namespace std;

class Graph {
    private:
        int rows, cols;
        int numVertices;
        int numEdges;
        int mapArray[NUM_ROWS][NUM_COLS];
        vector<vector<int> > adjList; // adjacency list of vertices

    public:
        // constructor for the graph with no items added
        Graph(int rows, int columns) {
            this->rows = rows;
            this->cols = columns;
            this->numVertices = rows * columns;
            this->numEdges = 0;
            this->adjList.clear();
            this->adjList.resize(rows * columns);
        }
    
    public:
        // adds an edge connection to the graph object
        void addEdge(int from, int to) {
            // check if edge has already been addded
            int count = 0;
            for (vector<int> listFrom: adjList) {
                for (int val: listFrom) {
                    if (count == from && val == to)  {
                        return; // duplicate item
                    }
                }
                count++;
            }
            
            // if not already added then add value
            adjList[from].push_back(to);
            this->numEdges += 1;
        }

    public:
        // creates a graph using the floodfill graph provided
        // returns the adjacency list of the graph
        void graphFF(char map[11][37]) {
            
            for (int i = 0; i < NUM_ROWS; i++) {
                for (int j = 0; j < NUM_COLS; j++) {
                    if (i > 0 && mapArray[i - 1][j] == mapArray[i][j] - 1) { // up
                        if (map[i*2][j * 4 + 2] != '-') { // check there is no wall in each direction
                            this->addEdge((i * cols) + j,((i - 1) * cols) + j);
                        }
                    }
                    if (j < cols && mapArray[i][j + 1] == mapArray[i][j] - 1) { // right
                        if (map[i*2 + 1][j * 4 + 4] != '|') {
                            this->addEdge((i * cols) + j,(i * cols) + j + 1);
                        }
                    }
                    if (i < rows && mapArray[i + 1][j] == mapArray[i][j] - 1) { // down
                        if (map[i*2 + 2][j * 4 + 2] != '-') {
                            this->addEdge((i * cols) + j,((i + 1) * cols) + j);
                        }
                    }
                    if (j > 0 && mapArray[i][j - 1] == mapArray[i][j] - 1) { // left
                        if (map[i*2 + 1][j * 4] != '|') {
                            this->addEdge((i * cols) + j,(i * cols) + j - 1);
                        }
                    }
                }
            }
        }

    public:
        // returns the adjacencylist for use in path planning
        vector<vector<int> > getAdjList() {
            return this->adjList;
        }  

    public:
        void createGraph(vector<int> startPos, vector<int> endPos, char textMap[11][37]) {
            floodFillMap(startPos, endPos, textMap);
            graphFF(textMap);
        }

    private:
        void printFloodFill() {
            for (int i = 0; i < NUM_ROWS; i++) {
                for (int j = 0; j < NUM_COLS; j++) {
                    cout << mapArray[i][j] << " ";
                }
                cout << endl;
            }

        }

    private:
        // performs a flood fill of the map
        void floodFillMap(vector<int> startPos, vector<int> endPos, char map[11][37]) {
            // flood fill
            // fill map Array with large number            
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
        }
};