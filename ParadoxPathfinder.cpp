
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
ipair operator-(const ipair&, const ipair&);
// Declare functions that will be used

inline int CalcIndex(ipair p1, int width);
inline int CalcCostH(ipair p1, ipair p2);
inline bool IsInsideBound(ipair cursor, pair<int, int> MapDimensions);
bool IsForcedNeighInLine(ipair origin, ipair dir, pair<int, int> Target, const vector<bool>& Map, vector<int>& parent, vector<int>& costG, pair<int, int> MapDimensions, ipair& FoundNeigh);

const ipair dUp(0, -1);
const ipair dDown(0, 1);
const ipair dLeft(-1, 0);
const ipair dRight(1, 0);

template<typename T>
void PrintVectorToSquare(vector<T>& vec, int width);

bool FindPath(pair<int, int> Start,
    pair<int, int> Target,
    const vector<bool>& Map,
    pair<int, int> MapDimensions,
    vector<int>& OutPath);
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
    int iStart = Start.first + Start.second * MapDimensions.first;
    int iTarget = Target.first + Target.second * MapDimensions.first;
    if (iStart == iTarget) return true;

    // Setup memory notepads
    vector<int> costG(Map.size(), Map.size() - 1);
    vector<int> parent(Map.size(), -1);

    // Setup p-queue
    auto compare = [](const qpair& lhs, const qpair& rhs)
    {
        return lhs.first > rhs.first;
    };
    priority_queue<qpair, vector<qpair>, decltype(compare)> p_frontier(compare);

    // Setup frontier and add starting point
    costG[iStart] = 0;
    p_frontier.emplace(qpair(0, Start));

    // Keep searching while valid frontier exists
    while (!p_frontier.empty()) {
        ipair cursor = p_frontier.top().second;
        int icursor = CalcIndex(cursor, MapDimensions.first);
        p_frontier.pop();

        //If cursor is goal, back-trace path and return true
        if (icursor == iTarget) {
            while (icursor != iStart) {
                OutPath.insert(OutPath.begin() + 0, icursor);
                icursor = parent[icursor];
            }
            return true;
        }

        // search for new forced neighs left first then right
        for (ipair dHorizontal : {dLeft, dRight}) {
            // On first iteration, include origin point
            ipair newCur = cursor +(dHorizontal == dLeft ? ipair(0, 0) : dRight);
            int inewCur = CalcIndex(newCur, MapDimensions.first);
            int lasticursor = icursor;

            while (IsInsideBound(newCur, MapDimensions) && Map[inewCur]) {
                // if we are at origin point no need to check this
                if (inewCur != icursor) {
                    if (!(costG[lasticursor] + 1 < costG[inewCur])) break;
                    costG[inewCur] = costG[lasticursor] + 1;
                    parent[inewCur] = lasticursor;
                }

                // scan upwards and downwards after forced neighbours
                for (ipair dVertical : {dUp, dDown}) {
                    ipair foundNeigh;
                    if (IsForcedNeighInLine(newCur, dVertical, Target, Map, parent, costG, MapDimensions, foundNeigh))
                        p_frontier.emplace(qpair(costG[CalcIndex(foundNeigh, MapDimensions.first)] + CalcCostH(foundNeigh, Target), foundNeigh));
                }

                // Check if space is target
                if (inewCur == iTarget)
                    p_frontier.emplace(qpair(costG[inewCur] + CalcCostH(newCur, Target), newCur));

                // keep stepping horizontally
                lasticursor = inewCur;
                newCur = newCur + dHorizontal;
                inewCur = CalcIndex(newCur, MapDimensions.first);
            }
        }
    }
    // If queue emptied not having reached Target, we return false
    return false;
}

bool IsForcedNeighInLine(ipair origin, ipair dir, pair<int, int> Target, const vector<bool>& Map, vector<int>& parent, vector<int>& costG, pair<int, int> MapDimensions, ipair& FoundNeigh) {
    ipair cursor = origin + dir;
    int iorigin = CalcIndex(origin, MapDimensions.first);
    int icursor = CalcIndex(cursor, MapDimensions.first);
    int lastiCursor = CalcIndex(origin, MapDimensions.first);

    // Keep searching for forced neigh in given direction while possible or found
    while (IsInsideBound(cursor, MapDimensions) && Map[icursor] && costG[lastiCursor] + 1 < costG[icursor]) {
        costG[icursor] = costG[lastiCursor] + 1;
        parent[icursor] = lastiCursor;
        // Considered to be forced neigh if we just passed a corner, or hit target
        if (Target == cursor
            ||
            (       IsInsideBound(cursor + dLeft - dir, MapDimensions)  && !Map[CalcIndex(cursor + dLeft - dir, MapDimensions.first)]
                &&  IsInsideBound(cursor + dLeft, MapDimensions)        &&  Map[CalcIndex(cursor + dLeft, MapDimensions.first)])
            ||
            (       IsInsideBound(cursor + dRight - dir, MapDimensions) && !Map[CalcIndex(cursor + dRight - dir, MapDimensions.first)]
                &&  IsInsideBound(cursor + dRight, MapDimensions)       &&  Map[CalcIndex(cursor + dRight, MapDimensions.first)])) {
            
                FoundNeigh = cursor;
                return true;
        }

        // keep stepping forward in direction
        lastiCursor = icursor;
        cursor = cursor + dir;
        icursor = CalcIndex(cursor, MapDimensions.first);
    }

    return false;
}

inline bool IsInsideBound(ipair cursor, pair<int, int> MapDimensions) {
    return (cursor.first >= 0 && cursor.first < MapDimensions.first&& cursor.second >= 0 && cursor.second < MapDimensions.second);
}

// Calculate the Manhattan-distance between two single-int coordinates in a grid.
inline int CalcCostH(ipair p1, ipair p2) {
    return abs(p1.first - p2.first) + abs(p1.second - p2.second);
}

inline int CalcIndex(ipair p, int width) {
    return p.first + p.second * width;
}

ipair operator+(const ipair& p1, const ipair& p2)
{
    return ipair(p1.first + p2.first, p1.second + p2.second);
}

ipair operator-(const ipair& p1, const ipair& p2)
{
    return ipair(p1.first - p2.first, p1.second - p2.second);
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


