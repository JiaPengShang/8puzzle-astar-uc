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

        out = s;//out is the new state after the move       
        int z = zeroPos(s);//z is the position of the blank space
        int r = z / 3, c = z % 3;  //r is the row, c is the column
        switch (mv)//mv is the move
        {
        case 'U':
            if (r == 0)//if the blank space is in the first row, return false

                return false;
            swap(out[z], out[z - 3]);
            return true;
        case 'R':

            if (c == 2)//if the blank space is in the last column, return false

                return false;
            swap(out[z], out[z + 1]);
            return true;
        case 'D':

            if (r == 2)//if the blank space is in the last row, return false        
                return false;
            swap(out[z], out[z + 3]);//swap the blank space with the space above it
            return true;
        case 'L':
            if (c == 0)//if the blank space is in the first column, return false

                return false;
            swap(out[z], out[z - 1]);
            return true;
        }
        return false;//if the move is invalid, return false
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

                    cnt++;//cnt is the number of misplaced tiles

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

                int v = s[i] - '0';//v is the value of the tile
                if (v == 0)
                    continue;
                int gi = posGoal[v];//gi is the goal position of the tile   
                int r1 = i / 3, c1 = i % 3, r2 = gi / 3, c2 = gi % 3; //r1 is the row of the tile, c1 is the column of the tile, r2 is the row of the goal position, c2 is the column of the goal position
                sum += std::abs(r1 - r2) + std::abs(c1 - c2); //sum is the sum of the manhattan distance

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

            const Node &A = (*nodes)[a];//A is the node at index a
            const Node &B = (*nodes)[b];//B is the node at index b

            if (A.f != B.f)
                return A.f > B.f; // larger f is worse
            return A.g < B.g;     // smaller g is worse → larger g is better
        }
    };

    string buildPath(const vector<Node> &nodes, int goalIdx)
    {
        string path;

        for (int cur = goalIdx; nodes[cur].parent != -1; cur = nodes[cur].parent)//cur is the current node
        {
            path.push_back(nodes[cur].move);//push the move to the path 

        }
        reverse(path.begin(), path.end());//reverse the path
        return path;
    }
}

// ================= Uniform Cost + Strict Expanded List =================
string uc_explist(string const initialState, string const goalState,
                  int &pathLength, int &numOfStateExpansions, int &maxQLength,
                  float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap,
                  int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions)
{

    clock_t t0 = clock();//t0 is the starting time  
    pathLength = 0;//pathLength is the length of the path
    numOfStateExpansions = 0;//numOfStateExpansions is the number of state expansions
    maxQLength = 0;//maxQLength is the maximum length of the queue
    numOfDeletionsFromMiddleOfHeap = 0;//numOfDeletionsFromMiddleOfHeap is the number of deletions from the middle of the heap
    numOfLocalLoopsAvoided = 0;//numOfLocalLoopsAvoided is the number of local loops avoided
    numOfAttemptedNodeReExpansions = 0;//numOfAttemptedNodeReExpansions is the number of attempted node re-expansions   

    if (initialState == goalState)
    {
        actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;//actualRunningTime is the actual running time
        return "";
    }

    vector<Node> nodes;//nodes is the vector of nodes
    nodes.reserve(100000);
    nodes.push_back(Node{initialState, 0, 0, 0, 0, -1});

    vector<int> heap;//heap is the vector of integers

    heap.reserve(100000);
    WorseByFG cmp{&nodes};
    heap.push_back(0);
    make_heap(heap.begin(), heap.end(), cmp);


    unordered_set<string> closed;//closed is the unordered set of strings
    closed.reserve(100003);
    unordered_map<string, int> openIndex;//openIndex is the unordered map of strings and integers
    openIndex.emplace(initialState, 0);

    string child;//child is the child state

    int goalIdx = -1;

    while (!heap.empty())
    {

        pop_heap(heap.begin(), heap.end(), cmp);//pop the heap
        int u = heap.back();
        heap.pop_back();
        openIndex.erase(nodes[u].s);//erase the state from the open index

        if (nodes[u].s == goalState)
        {
            goalIdx = u;//goalIdx is the index of the goal state

            break;
        }

        numOfStateExpansions++;//numOfStateExpansions is the number of state expansions
        closed.insert(nodes[u].s);

        for (char mv : MOVES)
        { // URDL

            if (!applyMove(nodes[u].s, mv, child))//if the move is invalid, continue
                continue;

            // Avoid local loops: do not return to the parent state
            if (nodes[u].parent != -1 && child == nodes[nodes[u].parent].s)//if the child state is the parent state, continue   
            {
                numOfLocalLoopsAvoided++;//numOfLocalLoopsAvoided is the number of local loops avoided
                continue;
            }
            // Strict Expanded List: skip states already expanded
            if (closed.count(child))//if the child state is already expanded, continue
            {
                numOfAttemptedNodeReExpansions++;//numOfAttemptedNodeReExpansions is the number of attempted node re-expansions
                continue;
            }
            // Strict: no decrease-key / reopen
            if (openIndex.count(child))//if the child state is already in the open index, continue
                continue;

            Node v;
            v.s = child;//v is the child state
            v.g = nodes[u].g + 1; // unit step cost = 1
            v.h = 0;//v.h is the heuristic value of the child state

            v.f = v.g;
            v.move = mv;
            v.parent = u;//v.parent is the parent state of the child state

            int vidx = (int)nodes.size();//vidx is the index of the child state
            nodes.push_back(v);
            heap.push_back(vidx);
            push_heap(heap.begin(), heap.end(), cmp);
            openIndex.emplace(v.s, vidx);//emplace the child state into the open index

            if ((int)heap.size() > maxQLength)

                maxQLength = (int)heap.size();//maxQLength is the maximum length of the queue

        }
    }

    string path;
    if (goalIdx != -1)
    {

        path = buildPath(nodes, goalIdx);//build the path from the nodes
        pathLength = (int)path.size();
    }
    else
    {
        pathLength = 0;//pathLength is the length of the path
    }
    actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;//actualRunningTime is the actual running time in seconds


    return path;
}

// ================= A* + Strict Expanded List =================
string aStar_ExpandedList(string const initialState, string const goalState,
                          int &pathLength, int &numOfStateExpansions, int &maxQLength,
                          float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap,
                          int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions,
                          heuristicFunction heuristic)
{

    clock_t t0 = clock();//t0 is the starting time              
    pathLength = 0;//pathLength is the length of the path
    numOfStateExpansions = 0;//numOfStateExpansions is the number of state expansions
    maxQLength = 0;//maxQLength is the maximum length of the queue
    numOfDeletionsFromMiddleOfHeap = 0;//numOfDeletionsFromMiddleOfHeap is the number of deletions from the middle of the heap
    numOfLocalLoopsAvoided = 0;//numOfLocalLoopsAvoided is the number of local loops avoided
    numOfAttemptedNodeReExpansions = 0;//numOfAttemptedNodeReExpansions is the number of attempted node re-expansions

    if (initialState == goalState)
    {
        actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;//actualRunningTime is the actual running time in seconds
        return "";
    }

    vector<Node> nodes;//nodes is the vector of nodes

    nodes.reserve(100000);
    int h0 = computeH(initialState, goalState, heuristic);
    nodes.push_back(Node{initialState, 0, h0, h0, 0, -1});


    vector<int> heap;//heap is the vector of integers

    heap.reserve(100000);
    WorseByFG cmp{&nodes};
    heap.push_back(0);
    make_heap(heap.begin(), heap.end(), cmp);


    unordered_set<string> closed;//closed is the unordered set of strings
    closed.reserve(100003);
    unordered_map<string, int> openIndex;//openIndex is the unordered map of strings and integers
    openIndex.emplace(initialState, 0);

    string child;//child is the child state

    int goalIdx = -1;

    while (!heap.empty())
    {

        pop_heap(heap.begin(), heap.end(), cmp);//pop the heap
        int u = heap.back();
        heap.pop_back();
        openIndex.erase(nodes[u].s);//erase the state from the open index

        if (nodes[u].s == goalState)
        {
            goalIdx = u;//goalIdx is the index of the goal state            

            break;
        }

        numOfStateExpansions++;//numOfStateExpansions is the number of state expansions
        closed.insert(nodes[u].s);

        for (char mv : MOVES)
        { // URDL

            if (!applyMove(nodes[u].s, mv, child))//if the move is invalid, continue
                continue;

            if (nodes[u].parent != -1 && child == nodes[nodes[u].parent].s)//if the child state is the parent state, continue
            {
                numOfLocalLoopsAvoided++;//numOfLocalLoopsAvoided is the number of local loops avoided
                continue;
            }
            if (closed.count(child))//if the child state is already expanded, continue
            {
                numOfAttemptedNodeReExpansions++;//numOfAttemptedNodeReExpansions is the number of attempted node re-expansions
                continue;
            }
            if (openIndex.count(child))//if the child state is already in the open index, continue
            {
                // Strict: no reopen / decrease-key
                continue;
            }

            Node v;
            v.s = child;//v is the child state
            v.g = nodes[u].g + 1;
            v.h = computeH(child, goalState, heuristic);
            v.f = v.g + v.h;
            v.move = mv;
            v.parent = u;//v.parent is the parent state of the child state

            int vidx = (int)nodes.size();//vidx is the index of the child state
            nodes.push_back(v);
            heap.push_back(vidx);
            push_heap(heap.begin(), heap.end(), cmp);
            openIndex.emplace(v.s, vidx);//emplace the child state into the open index

            if ((int)heap.size() > maxQLength)

                maxQLength = (int)heap.size();//maxQLength is the maximum length of the queue

        }
    }

    string path;
    if (goalIdx != -1)
    {

        path = buildPath(nodes, goalIdx);//build the path from the nodes    
        pathLength = (int)path.size();
    }
    else
    {
        pathLength = 0;//pathLength is the length of the path
    }
    actualRunningTime = float(clock() - t0) / CLOCKS_PER_SEC;//actualRunningTime is the actual running time in seconds      

    return path;
}
