#include "algorithm.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <string>
#include <ctime>
#include <cmath>

using namespace std;

namespace
{
    struct Node
    {
        string s;        // "012345678"
        int g = 0;       // cost so far
        int h = 0;       // heuristic
        int f = 0;       // g + h
        char move = 0;   // 'U','R','D','L'
        int parent = -1; // index in nodes
    };

    // Fixed successor order: URDL (Up, Right, Down, Left)
    static const char MOVES[4] = {'U', 'R', 'D', 'L'};

    inline int zeroPos(const string &s) { return (int)s.find('0'); }

    inline bool applyMove(const string &s, char mv, string &out)
    {
        out = s;
        int z = zeroPos(s);
        int r = z / 3, c = z % 3;
        switch (mv)
        {
        case 'U':
            if (r == 0)
                return false;
            swap(out[z], out[z - 3]);
            return true;
        case 'R':
            if (c == 2)
                return false;
            swap(out[z], out[z + 1]);
            return true;
        case 'D':
            if (r == 2)
                return false;
            swap(out[z], out[z + 3]);
            return true;
        case 'L':
            if (c == 0)
                return false;
            swap(out[z], out[z - 1]);
            return true;
        }
        return false;
    }

    // Compute heuristic (note: Misplaced Tiles ignores tile 0)
    int computeH(const string &s, const string &goal, heuristicFunction hf)
    {
        if (hf == misplacedTiles)
        {
            int cnt = 0;
            for (int i = 0; i < 9; i++)
            {
                char v = s[i];
                if (v != '0' && v != goal[i])
                    cnt++;
            }
            return cnt;
        }
        else
        { // manhattanDistance
            int sum = 0;
            int posGoal[10];
            for (int i = 0; i < 9; i++)
                posGoal[goal[i] - '0'] = i; // value -> index
            for (int i = 0; i < 9; i++)
            {
                int v = s[i] - '0';
                if (v == 0)
                    continue;
                int gi = posGoal[v];
                int r1 = i / 3, c1 = i % 3, r2 = gi / 3, c2 = gi % 3;
                sum += std::abs(r1 - r2) + std::abs(c1 - c2);
            }
            return sum;
        }
    }

    // Min-heap comparator: smaller f first; if f ties, prefer larger g (tie-break on larger g)
    struct WorseByFG
    {
        const vector<Node> *nodes;
        bool operator()(int a, int b) const
        {
            const Node &A = (*nodes)[a];
            const Node &B = (*nodes)[b];
            if (A.f != B.f)
                return A.f > B.f; // larger f is worse
            return A.g < B.g;     // smaller g is worse → larger g is better
        }
    };

    string buildPath(const vector<Node> &nodes, int goalIdx)
    {
        string path;
        for (int cur = goalIdx; nodes[cur].parent != -1; cur = nodes[cur].parent)
        {
            path.push_back(nodes[cur].move);
        }
        reverse(path.begin(), path.end());
        return path;
    }
}

// ================= Uniform Cost + Strict Expanded List =================
string uc_explist(string const initialState, string const goalState,
                  int &pathLength, int &numOfStateExpansions, int &maxQLength,
                  float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap,
                  int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions)
{
    clock_t t0 = clock();
    pathLength = 0;
    numOfStateExpansions = 0;
    maxQLength = 0;
    numOfDeletionsFromMiddleOfHeap = 0;
    numOfLocalLoopsAvoided = 0;
    numOfAttemptedNodeReExpansions = 0;

    if (initialState == goalState)
    {
        actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;
        return "";
    }

    vector<Node> nodes;
    nodes.reserve(100000);
    nodes.push_back(Node{initialState, 0, 0, 0, 0, -1});

    vector<int> heap;
    heap.reserve(100000);
    WorseByFG cmp{&nodes};
    heap.push_back(0);
    make_heap(heap.begin(), heap.end(), cmp);

    unordered_set<string> closed;
    closed.reserve(100003);
    unordered_map<string, int> openIndex;
    openIndex.emplace(initialState, 0);

    string child;
    int goalIdx = -1;

    while (!heap.empty())
    {
        pop_heap(heap.begin(), heap.end(), cmp);
        int u = heap.back();
        heap.pop_back();
        openIndex.erase(nodes[u].s);

        if (nodes[u].s == goalState)
        {
            goalIdx = u;
            break;
        }

        numOfStateExpansions++;
        closed.insert(nodes[u].s);

        for (char mv : MOVES)
        { // URDL
            if (!applyMove(nodes[u].s, mv, child))
                continue;

            // Avoid local loops: do not return to the parent state
            if (nodes[u].parent != -1 && child == nodes[nodes[u].parent].s)
            {
                numOfLocalLoopsAvoided++;
                continue;
            }
            // Strict Expanded List: skip states already expanded
            if (closed.count(child))
            {
                numOfAttemptedNodeReExpansions++;
                continue;
            }
            // Strict: no decrease-key / reopen
            if (openIndex.count(child))
                continue;

            Node v;
            v.s = child;
            v.g = nodes[u].g + 1; // unit step cost = 1
            v.h = 0;
            v.f = v.g;
            v.move = mv;
            v.parent = u;

            int vidx = (int)nodes.size();
            nodes.push_back(v);
            heap.push_back(vidx);
            push_heap(heap.begin(), heap.end(), cmp);
            openIndex.emplace(v.s, vidx);

            if ((int)heap.size() > maxQLength)
                maxQLength = (int)heap.size();
        }
    }

    string path;
    if (goalIdx != -1)
    {
        path = buildPath(nodes, goalIdx);
        pathLength = (int)path.size();
    }
    else
    {
        pathLength = 0;
    }
    actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;
    return path;
}

// ================= A* + Strict Expanded List =================
string aStar_ExpandedList(string const initialState, string const goalState,
                          int &pathLength, int &numOfStateExpansions, int &maxQLength,
                          float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap,
                          int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions,
                          heuristicFunction heuristic)
{
    clock_t t0 = clock();
    pathLength = 0;
    numOfStateExpansions = 0;
    maxQLength = 0;
    numOfDeletionsFromMiddleOfHeap = 0;
    numOfLocalLoopsAvoided = 0;
    numOfAttemptedNodeReExpansions = 0;

    if (initialState == goalState)
    {
        actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;
        return "";
    }

    vector<Node> nodes;
    nodes.reserve(100000);
    int h0 = computeH(initialState, goalState, heuristic);
    nodes.push_back(Node{initialState, 0, h0, h0, 0, -1});

    vector<int> heap;
    heap.reserve(100000);
    WorseByFG cmp{&nodes};
    heap.push_back(0);
    make_heap(heap.begin(), heap.end(), cmp);

    unordered_set<string> closed;
    closed.reserve(100003);
    unordered_map<string, int> openIndex;
    openIndex.emplace(initialState, 0);

    string child;
    int goalIdx = -1;

    while (!heap.empty())
    {
        pop_heap(heap.begin(), heap.end(), cmp);
        int u = heap.back();
        heap.pop_back();
        openIndex.erase(nodes[u].s);

        if (nodes[u].s == goalState)
        {
            goalIdx = u;
            break;
        }

        numOfStateExpansions++;
        closed.insert(nodes[u].s);

        for (char mv : MOVES)
        { // URDL
            if (!applyMove(nodes[u].s, mv, child))
                continue;

            if (nodes[u].parent != -1 && child == nodes[nodes[u].parent].s)
            {
                numOfLocalLoopsAvoided++;
                continue;
            }
            if (closed.count(child))
            {
                numOfAttemptedNodeReExpansions++;
                continue;
            }
            if (openIndex.count(child))
            {
                // Strict: no reopen / decrease-key
                continue;
            }

            Node v;
            v.s = child;
            v.g = nodes[u].g + 1;
            v.h = computeH(child, goalState, heuristic);
            v.f = v.g + v.h;
            v.move = mv;
            v.parent = u;

            int vidx = (int)nodes.size();
            nodes.push_back(v);
            heap.push_back(vidx);
            push_heap(heap.begin(), heap.end(), cmp);
            openIndex.emplace(v.s, vidx);

            if ((int)heap.size() > maxQLength)
                maxQLength = (int)heap.size();
        }
    }

    string path;
    if (goalIdx != -1)
    {
        path = buildPath(nodes, goalIdx);
        pathLength = (int)path.size();
    }
    else
    {
        pathLength = 0;
    }
    actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;
    return path;
}
