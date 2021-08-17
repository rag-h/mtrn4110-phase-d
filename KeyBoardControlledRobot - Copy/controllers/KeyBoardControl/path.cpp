// C++ program to print all paths of source to
// destination in given graph
// heavily modified from code: https://www.geeksforgeeks.org/print-paths-given-source-destination-using-bfs/

#include <iostream>
#include <cmath>

using namespace std;

// converts a position in a matrix into coordinates as a vector
vector<int> posToCoord(int pos, int numRows, int numCols) {
	vector<int> returnCoord;
	if (pos == 0) {
		returnCoord.push_back(0);
		returnCoord.push_back(0);
		return returnCoord;
	}
	returnCoord.push_back(floor(pos / numCols)); // row
	returnCoord.push_back(pos - (numCols * returnCoord[0])); // col
	return returnCoord;
}

// prints out a path
void printpath(vector<int> path) {	
	int size = path.size();
	for (int i = 0; i < size; i++)
		cout << path[i] << " ";
	cout << endl;
}

// check if a current position is in the path already
// true if it's already in the path, false if not
bool isNotVisited(int x, vector<int>& path) {
	int size = path.size();
	for (int i = 0; i < size; i++)
		if (path[i] == x)
			return false;
	return true;
}

// finds and prints all paths from source to destination
vector<vector<int> > findpaths(vector<vector<int> > g, int src, int dst) {
	vector<vector<int> > paths;
	// queue to store paths
	queue<vector<int> > q;
	// current path
	vector<int> currPath;
	// add initial items to path & queue
	currPath.push_back(src); 
	q.push(currPath);
	while (!q.empty()) {
		currPath = q.front();
		q.pop();
		int prev = currPath[currPath.size() - 1];
		// if this vertex is the destination then path is complete
		if (prev == dst) paths.push_back(currPath);

		// loop through all vertexes connected to the current vertex, add them to queue
		int sizePrev = g[prev].size();
		for (int i = 0; i < sizePrev; i++) {
			if (isNotVisited(g[prev][i], currPath)) {
				vector<int> newpath(currPath);
				newpath.push_back(g[prev][i]);
				q.push(newpath);
			}
		}
	}
	return paths;
}

vector<vector<vector<int> > > getPathCoords(vector<vector<int> > g, int src, int dst,
										int numRows, int numCols) {
	vector<vector<vector<int> > > pathListCoords;
	vector<vector<vector<int> > > pathListPos;
	vector<vector<int> > paths = findpaths(g, src, dst);
	for (vector<int> path: paths) {
		vector<vector<int> > newPath;
		for (int pos: path) {
			vector<int> coord = posToCoord(pos, numRows, numCols);
			newPath.push_back(posToCoord(pos, numRows, numCols));
		}
		pathListCoords.push_back(newPath);
	}
	return pathListCoords;
}