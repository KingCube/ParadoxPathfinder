
#include <iostream>
#include <iomanip>
#include <chrono>
#include <queue>
#include <climits>
#include <tuple>
#include <stdlib.h>

using namespace std;

// Small typedefs for ease of reading
typedef pair<int, int> ipair;
typedef pair<int, ipair> qpair;

ipair operator+(const ipair&, const ipair&);

// Declare functions that will be used
bool FindPath(pair<int, int> Start,
    pair<int, int> Target,
    const vector<bool>& Map,
    pair<int, int> MapDimensions,
    vector<int>& OutPath);

inline int CalcPos(ipair p1, int width);
inline int CalcDistance(ipair p1, ipair p2);

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
    cout << "pathlenght: " << outPath.size() << endl;
    for (int cur : outPath)
        cout << cur << endl;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto time = end_time - start_time;

    std::cout << "program took " <<
        time / std::chrono::milliseconds(1) << "ms to run.\n";
}

bool FindPath(pair<int, int> Start,
    pair<int, int> Target,
    const vector<bool>& Map,
    pair<int, int> MapDimensions,
    vector<int>& OutPath) {

    // translate input pos's to single-int coordinates
    int iStart =    Start.first     + Start.second * MapDimensions.first;
    int iTarget =   Target.first    + Target.second * MapDimensions.first;
    if (iStart == iTarget) return true;

    // Setup memory notepads
    vector<int> costG(Map.size(), Map.size() - 1);
    vector<int> parent(Map.size(), -1);
    vector<bool> closed(Map.size(), false);
    vector<bool> observed(Map.size(), false);

    // Setup p-queue comparer
    auto compare = [](const qpair& lhs, const qpair& rhs)
    {
        return lhs.first > rhs.first;
    };
    priority_queue<qpair, vector<qpair>, decltype(compare)> p_frontier(compare);

    // Setup frontier and add starting point
    costG[iStart] = 0;
    p_frontier.emplace(qpair(0, Start));

    // neighboor space
    ipair dirs[4]{ ipair(-1, 0), ipair(1, 0), ipair(0, -1), ipair(0, 1) };
    bool neighValid[4];

    // Keep searching while valid frontier exists
    while (!p_frontier.empty()) {
        ipair cursor = p_frontier.top().second;
        p_frontier.pop();
        int icursor = CalcPos(cursor, MapDimensions.first);

        //If already evaluated, skip
        if (closed[icursor]) continue;
        closed[icursor] = true;

        //If cursor is goal, back-trace path and return true
        if (icursor == iTarget) {
            while (icursor != iStart) {
                OutPath.insert(OutPath.begin() + 0, icursor);
                icursor = parent[icursor];
            }
            return true;
        }

        //Check for valid neighboor-directions
        neighValid[0] = cursor.first != 0;
        neighValid[1] = cursor.first != MapDimensions.first -1;
        neighValid[2] = cursor.second != 0;
        neighValid[3] = cursor.second != MapDimensions.second - 1;

        //Enqueue neighboors if qualified
        for (int i = 0; i < 4; i++) {
            if (!neighValid[i]) continue;
            ipair newCur = cursor + dirs[i];
            int inewCur = CalcPos(newCur, MapDimensions.first);

            if (!closed[inewCur] && !observed[inewCur] && Map[inewCur] && costG[icursor] + 1 < costG[inewCur]) {
                costG[inewCur] = costG[icursor] + 1;
                int costH = CalcDistance(newCur, Target);
                p_frontier.emplace(qpair(costG[inewCur] + costH, newCur));
                parent[inewCur] = icursor;
                observed[inewCur] = true;
            }
        }
    }
    // If queue emptied not having reached Target, we return false
    return false;
}

inline int CalcDistance(ipair p1, ipair p2) {
    // Calculate the Manhattan-distance between two single-int coordinates in a grid.
    return abs(p1.first-p2.first) + abs(p1.second-p2.second);
}

inline int CalcPos(ipair p, int width) {
    return p.first + p.second * width;
}

ipair operator+(const ipair& p1, const ipair& p2)
{
    return ipair(p1.first+p2.first,p1.second+p2.second);
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


