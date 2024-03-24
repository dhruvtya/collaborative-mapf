#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

#include "iostream"
#include "vector"
#include "queue"
#include "memory"

using namespace std;

typedef vector<vector<int>> Map;

struct Node;

const vector<pair<int, int>> valid_moves_2d = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}, {0, 0}};

void computeHeuristics(const Map& obstacle_map, pair<int, int> goal, Map& heuristic_map);

#endif // HEURISTICS_HPP
