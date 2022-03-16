// ParadoxPathfinder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <iomanip>
#include <queue>
#include <climits>
#include <tuple>

using namespace std;

// Small typedef for ease of reading
typedef pair<int, int> ipair;

// Declare functions that will be used
bool FindPath(pair<int, int> Start,
    pair<int, int> Target,
    const vector<bool>& Map,
    pair<int, int> MapDimensions,
    vector<int>& OutPath);

int CalcDistance(int c1, int c2, int width);

template<typename T>
void PrintVectorToSquare(vector<T>& vec, int width);


int main()
{
    vector<bool> map = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
        1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1,
        1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1,
        1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1,
        1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    };

    ipair Start(5, 11);
    ipair Target(5, 1);
    ipair MapDims(11, 12);

    vector<int> outPath;

    //PrintVectorToSquare<bool>(map, MapDims.first);

    bool hej = FindPath(Start, Target, map, MapDims, outPath);
    for (int cur : outPath)
        cout << cur << endl;
}

bool FindPath(  pair<int, int> Start,
                pair<int, int> Target,
                const vector<bool>& Map,
                pair<int, int> MapDimensions,
                vector<int>& OutPath) {

    // Setup memory notepads
    vector<int> costF(Map.size(),INT_MAX);
    vector<int> costG(Map.size(),INT_MAX);
    vector<int> parent(Map.size(), -1);
    vector<bool> visited(Map.size(), false);
    
    // translate input pos's to single-int coordinates
    int iStart  = Start.first   + Start.second* MapDimensions.first;
    int iTarget = Target.first  + Target.second* MapDimensions.first;

    // Setup frontier and add starting point
    priority_queue<ipair> p_frontier;
    
    costG[iStart] = 0;
    costF[iStart] = CalcDistance(iStart, iTarget, MapDimensions.first);
    p_frontier.emplace(ipair(-costF[iStart], iStart)); // use minus as priority --> largest element

    // neighboor space
    queue<int> neigh;

    // Keep searching while valid frontier exists
    while (!p_frontier.empty()) {
        int cursor = p_frontier.top().second;

        //If cursor is goal, back-trace path and return true
        if (cursor == iTarget) {
            while (cursor != iStart) {
                OutPath.insert(OutPath.begin() + 0,cursor);
                cursor = parent[cursor];
            }   
            // DEBUG only: Print memory-matrices contents
            //PrintVectorToSquare<int>(costG, MapDimensions.second);
            //PrintVectorToSquare<int>(costF, MapDimensions.second);
            //PrintVectorToSquare<bool>(visited, MapDimensions.second);
            return true;
        }
        p_frontier.pop();

        //If already evaluated, skip
        if (visited[cursor] && (visited[cursor] = true)) continue;

        //Check for valid neighboor-directions
        if (cursor % MapDimensions.first != 0)                             neigh.push(cursor - 1);
        if (cursor % MapDimensions.first != MapDimensions.first - 1)       neigh.push(cursor + 1);
        if (cursor / MapDimensions.first != 0)                             neigh.push(cursor - MapDimensions.first);
        if (cursor / MapDimensions.first != MapDimensions.second - 1)      neigh.push(cursor + MapDimensions.first);

        //Enqueue neighboors if qualified
        while(!neigh.empty()){
            int newCur = neigh.front(); neigh.pop();
            if (!visited[newCur] && Map[newCur] && costG[cursor] + 1 < costG[newCur]) {
                costG[newCur] = costG[cursor] + 1;
                costF[newCur] = costG[newCur] + CalcDistance(newCur, iTarget, MapDimensions.first);
                parent[newCur] = cursor;
                p_frontier.emplace(ipair(-costF[newCur], newCur));
            }
        }
    }

    // DEBUG only: Print memory-matrices contents
    //PrintVectorToSquare<int>(costG, MapDimensions.second);
    //PrintVectorToSquare<int>(costF, MapDimensions.second);
    //PrintVectorToSquare<bool>(visited, MapDimensions.second);

    // If queue emptied not having reached Target, we return false
    return false;
}

inline int CalcDistance(int c1, int c2, int width) {
    // Calculate the Manhattan-distance between two single-int coordinates in a grid.
    return abs(c1 / width - c2 / width) + abs(c1 % width - c2 % width);
}

template<typename T>
void PrintVectorToSquare(vector<T>& vec, int width)
{
    cout << "****** Begin printing vector as matrix ******" << endl;
    int cursor = 0;
    for (int r = 0; r < vec.size() / width; r++) {
        for (int c = 0; c < width; c++)
            cout << setw(16) << vec[cursor++];
        cout << endl;
    }
}
