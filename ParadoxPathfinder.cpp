// ParadoxPathfinder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <iomanip>
#include <chrono>
#include <queue>
#include <climits>
#include <tuple>
#include <stdlib.h>

using namespace std;

// Small typedef for ease of reading
typedef tuple<int, int, int> itriplet;

// Declare functions that will be used
bool FindPath(pair<int, int> Start,
    pair<int, int> Target,
    const vector<bool>& Map,
    pair<int, int> MapDimensions,
    vector<int>& OutPath);

inline int CalcDistance(int c1, int c2, int width);

template<typename T>
void PrintVectorToSquare(vector<T>& vec, int width);

int main()
{
    
    vector<bool> map = {
        1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1,
        1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1,
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
    pair<int, int> Start(5, 11);
    pair<int, int> Target(5, 1);
    pair<int, int> MapDims(11, 12);
    

    /*
    vector<bool> map = { 
        1,0,0,0,0,0,0,1};
    
    pair<int,int> Start(0, 0);
    pair<int, int> Target(7, 0);
    pair<int, int> MapDims(8, 1);
    */

    vector<int> outPath;
    
    //PrintVectorToSquare<bool>(map, MapDims.first);
    auto start_time = std::chrono::high_resolution_clock::now();
    bool hej = FindPath(Start, Target, map, MapDims, outPath);

    cout << "answer was " << hej << endl;
    for (int cur : outPath)
        cout << cur << endl;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto time = end_time - start_time;

    std::cout << "program took " <<
        time / std::chrono::milliseconds(1) << "ms to run.\n";
}

bool FindPath(  pair<int, int> Start,
                pair<int, int> Target,
                const vector<bool>& Map,
                pair<int, int> MapDimensions,
                vector<int>& OutPath) {
    
    // translate input pos's to single-int coordinates
    int iStart  = Start.first   + (int)Start.second* MapDimensions.first;
    int iTarget = Target.first  + (int)Target.second* MapDimensions.first;
    if (iStart == iTarget) return true;

    // Setup memory notepads, pointers's a little faster than vectors
    int* costF = new int[Map.size()]; fill(costF, costF + Map.size(), INT_MAX);
    int* costG = new int[Map.size()]; fill(costG, costG + Map.size(), INT_MAX);
    int* parent = new int[Map.size()]; fill(parent, parent + Map.size(), -1);
    vector<bool> closed(Map.size(), false);
    
    // Setup p-queue comparer, tie-breaker as steps to goal.
    auto compare = [](const itriplet &lhs, const itriplet &rhs)
    {
        return get<0>(lhs) > get<0>(rhs) || (get<0>(lhs) == get<0>(rhs) && get<1>(lhs) > get<1>(rhs));
    };
    priority_queue<itriplet, vector<itriplet>, decltype(compare)> p_frontier(compare);
    
    // Setup frontier and add starting point
    int round = 0;
    costG[iStart] = 0;
    costF[iStart] = CalcDistance(iStart, iTarget, MapDimensions.first);
    p_frontier.emplace(itriplet(costF[iStart], round++, iStart));

    // neighboor space
    int neigh[4]{ -1,1, -MapDimensions.first, MapDimensions.first };
    bool neighValid[4];

    // Keep searching while valid frontier exists
    while (!p_frontier.empty()) {
        int cursor = get<2>(p_frontier.top());
        p_frontier.pop();

        //If already evaluated, skip
        if (closed[cursor]) continue;
        closed[cursor] = true;

        //If cursor is goal, back-trace path and return true
        if (cursor == iTarget) {
            while (cursor != iStart) {
                OutPath.insert(OutPath.begin() + 0, cursor);
                cursor = parent[cursor];
            }
            delete[] costF; delete[] costG; delete[] parent;
            return true;
        }

        //Check for valid neighboor-directions
        neighValid[0] = cursor % MapDimensions.first != 0;
        neighValid[1] = cursor % MapDimensions.first != MapDimensions.first - 1;
        neighValid[2] = cursor / MapDimensions.first != 0;
        neighValid[3] = cursor / MapDimensions.first != MapDimensions.second - 1;

        //Enqueue neighboors if qualified
        for (int i = 0; i < 4; i++) {
            if (!neighValid[i]) continue;
            int newCur = cursor + neigh[i];
            if (!closed[newCur] && Map[newCur] && costG[cursor] + 1 < costG[newCur]) {
                costG[newCur] = costG[cursor] + 1;
                int costH = CalcDistance(newCur, iTarget, MapDimensions.first);
                costF[newCur] = costG[newCur] + costH;
                parent[newCur] = cursor;
                p_frontier.emplace(itriplet(costF[newCur], costH, newCur));
            }
        }
    }
    // If queue emptied not having reached Target, we return false
    delete[] costF; delete[] costG; delete[] parent;
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
