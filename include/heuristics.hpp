#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

#include "iostream"
#include "vector"
#include "queue"
#include "memory"

using namespace std;

/**
 * @brief Typedef for the map
 */
typedef vector<vector<int>> Map;

/**
 * @brief Struct for the node
 */
struct Node;

// Vector of valid moves for the robot
const vector<pair<int, int>> valid_moves_2d = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}, {0, 0}};

/**
 * @brief Compute the heuristics for the given goal and map for an agent
 * 
 * @param obstacle_map The map of the environment
 * @param goal The goal of the agent
 * @param heuristic_map The heuristics to be returned
*/
void computeHeuristics(const Map& obstacle_map, pair<int, int> goal, Map& heuristic_map);

#endif // HEURISTICS_HPP
