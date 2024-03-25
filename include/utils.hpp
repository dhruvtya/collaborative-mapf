#ifndef UTILS_HPP
#define UTILS_HPP

#include "iostream"
#include "vector"
#include "fstream" 

using namespace std;

enum class AgentType{
    TRANSIT = 0,
    HELPER = 1
};

struct Result{
    int agent_id_;
    AgentType type_;
    pair<int, int> start_;
    pair<int, int> goal_;
    vector<pair<int, int>> path_;
};

namespace utils{
    bool loadMap(string &filename, vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, vector<pair<int, int>> &helper_parkings);
    void printMap(const vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, vector<pair<int, int>> &helper_parkings);
    void printHeuristicMap(const vector<vector<int>> &map);
    void printResults(const vector<Result> &results);
    void saveSolution(const vector<Result> &results, string filename);
}

#endif // UTILS_HPP
