#ifndef UTILS_HPP
#define UTILS_HPP

#include "iostream"
#include "vector"
#include "fstream" 

using namespace std;

namespace utils{
    bool loadMap(string &filename, vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, pair<int, int> &helper_parking);
    void printMap(const vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, pair<int, int> &helper_parking);
    void saveSolution();
}

#endif // UTILS_HPP
