#include "heuristics.hpp"

struct HNode {
    int x;
    int y;
    int dist;
};

class CompareHNodes
{
public:
    bool operator() (HNode lhs, HNode rhs)
    {
        return lhs.dist > rhs.dist;
    }
};

void computeHeuristics(const Map& obstacle_map, pair<int, int> goal, Map& heuristic_map){

    int x_size = obstacle_map.size();
    int y_size = obstacle_map[0].size();

    vector<vector<bool>> visited;

    // heuristics
    for (int i = 0; i < x_size; i++) {
        vector<int> heuristic_initialize(y_size, INT32_MAX);
        vector<bool> visited_initialize(y_size, false);
        heuristic_map.push_back(heuristic_initialize);
        visited.push_back(visited_initialize);
    }

    priority_queue<HNode, vector<HNode>, CompareHNodes> open_list;    

    heuristic_map[goal.first][goal.second] = 0;
    HNode start_node = {goal.first, goal.second, 0};
    open_list.push(start_node);

    while (!open_list.empty()) {
        HNode curr_node = open_list.top();
        open_list.pop();

        if (visited[curr_node.x][curr_node.y]) {
            continue;
        } else {
            visited[curr_node.x][curr_node.y] = true;
        }

        heuristic_map[curr_node.x][curr_node.y] = curr_node.dist;

        for (auto move:valid_moves_2d) {
            HNode new_node = {curr_node.x + move.first, curr_node.y + move.second, curr_node.dist + 1};

            if (new_node.x < 0 || new_node.x >= x_size || new_node.y < 0 || new_node.y >= y_size) {
                continue;
            }

            bool has_obstacle = (obstacle_map[new_node.x][new_node.y] == -1);

            if (has_obstacle || visited[new_node.x][new_node.y]) {
                continue;
            }

            open_list.push(new_node);
        }
    }
}