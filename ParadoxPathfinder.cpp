// ParadoxPathfinder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <iomanip>
#include <chrono>
#include <climits>
#include <tuple>
#include <stdlib.h>
#include <unordered_set>
#include <vector>

using namespace std;

// Declare functions that will be used
bool FindPath(pair<int, int> Start,
    pair<int, int> Target,
    const vector<bool>& Map,
    pair<int, int> MapDimensions,
    vector<int>& OutPath);

int main()
{
    /*
    vector<bool> map = {
        1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1,
        1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1,
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
    */

    
    vector<bool> map = {
        1, 1, 1, 1,
        0, 1, 0, 1,
        0, 1, 1, 1
    };
    
    pair<int,int> Start(0, 0);
    pair<int, int> Target(1, 2);
    pair<int, int> MapDims(4, 3);

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

    // Setup memory notepads
    unordered_set<int> frontiers[2];
    unordered_set<int> newfrontiers[2];
    unordered_set<int> visiteds[2];
    vector<int> parents[2]{ vector<int>(Map.size(),-1),vector<int>(Map.size(),-1) };

    // Setup frontier and add starting point
    frontiers[0].insert(iStart);
    frontiers[1].insert(iTarget);

    // neighboor space
    int neigh[4]{ -1,1, -MapDimensions.first, MapDimensions.first };
    bool neighValid[4];

    bool fromStart = true;

    // Keep searching while valid frontier exists
    while (!frontiers[0].empty() && !frontiers[1].empty()) {
        //If cursor is goal, back-trace path and return true
        for(int cursor : frontiers[fromStart])
        {
            if (visiteds[fromStart].find(cursor) != visiteds[fromStart].end()) continue; //borde inte behövas?
            visiteds[fromStart].insert(cursor);

            // check if searches have found eachother
            if (visiteds[!fromStart].find(cursor) != visiteds[!fromStart].end()) {
                int pather = cursor;
                while (pather != -1) {
                    OutPath.push_back(pather);
                    pather = parents[1][pather];
                }

                pather = parents[0][cursor];
                while (pather != iStart && pather != -1) {
                    OutPath.insert(OutPath.begin(), pather);
                    pather = parents[0][pather];
                }
                return true;
            }
            //Check for valid neighboor-directions
            neighValid[0] = cursor % MapDimensions.first != 0;
            neighValid[1] = cursor % MapDimensions.first != MapDimensions.first - 1;
            neighValid[2] = cursor / MapDimensions.first != 0;
            neighValid[3] = cursor / MapDimensions.first != MapDimensions.second - 1;
            
            for (int i = 0; i < 4; i++) {
                if (!neighValid[i]) continue;
                int newCursor = cursor + neigh[i];
                if (Map[newCursor]
                    && visiteds[fromStart].find(newCursor) == visiteds[fromStart].end()
                    && newfrontiers[fromStart].find(newCursor) == newfrontiers[fromStart].end()) 
                {
                    newfrontiers[fromStart].insert(newCursor);
                    parents[fromStart][newCursor] = cursor;
                }
            }
        }
        
        frontiers[fromStart].clear();
        frontiers[fromStart].insert(newfrontiers[fromStart].begin(), newfrontiers[fromStart].end());
        newfrontiers[fromStart].clear();
        fromStart = !fromStart;
    }
    // If queue emptied not having reached Target, we return false
    return false;
}
