/* implements a graph class structure for creating the graph
    Author: Luke Jackson
    heavily adpated / inspired from these sources:
    https://www.techiedelight.com/graph-implementation-using-stl/ <- most inspiration
    https://www.geeksforgeeks.org/graph-and-its-representations/
    https://www.softwaretestinghelp.com/graph-implementation-cpp/
*/

#include <iostream>
#include <vector>

using namespace std;

class Graph {
    private:
        int rows, cols;
        int numVertices;
        int numEdges;
        vector<vector<int>> adjList; // adjacency list of vertices

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
        void graphFF(int rows, int cols, int mapArray[5][9], int mapRows, int mapCols, char map[11][37]) {
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
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
        vector<vector<int>> getAdjList() {
            return this->adjList;
        }  
};