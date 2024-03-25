#include "a_star.hpp"

#define GETMAPINDEX(X, Y, T, XSIZE, YSIZE) (T * XSIZE * YSIZE + (Y - 1) * XSIZE + (X - 1))


/**
 * @brief Build the constraint table for a given agent for A* search
 * 
 * @param constraints - List of constraints
 * @param agent_id - ID of the agent for which the constraint table is being built
 * @param constraint_table - Reference to the constraint table [Return]
 */
void AStar::buildConstraintTable(const vector<Constraint>& constraints, const int& agent_id, ConstraintTable& constraint_table){
    // Clear the constraint table
    constraint_table.clear();

    // Add constraints to the constraint table
    for (auto constraint:constraints) {
        if (constraint.agent_id == agent_id) {
            if (constraint_table.find(constraint.time_step) == constraint_table.end()) {
                vector<Constraint> constraint_list;
                constraint_list.push_back(constraint);
                constraint_table[constraint.time_step] = constraint_list;
            } else {
                constraint_table[constraint.time_step].push_back(constraint);
            }
        }
    }
}

/**
 * @brief Check if a location is within the map bounds
 * 
 * @param location 
 * @param x_size 
 * @param y_size 
 * @return true 
 * @return false 
 */
bool AStar::inMap(const pair<int, int>& location, const int& x_size, const int& y_size){
    return (location.first >= 0 && location.first < x_size && location.second >= 0 && location.second < y_size);
}

/**
 * @brief Check if a move is constrained by the constraint table
 * 
 * @param curr_location 
 * @param next_location 
 * @param next_time 
 * @param constraint_table 
 * @return true 
 * @return false 
 */
bool AStar::isConstrained(const pair<int, int>& curr_location, const pair<int, int>& next_location, const int& next_time, const ConstraintTable& constraint_table){
    if (constraint_table.find(next_time) != constraint_table.end()) {
        for (auto constraint:constraint_table.at(next_time)) {
            if (constraint.location[0] == next_location) {
                return true;
            }
            if (constraint.location.size() > 1) {
                if(constraint.location[0] == curr_location && constraint.location[1] == next_location){
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief Get the path from the current node
 * 
 * @param current_node 
 * @param path 
 */
void AStar::getPath(const Map& obstacle_map, const AgentType& agent_type, const shared_ptr<Node>& current_node, vector<pair<int, int>>& path, vector<pair<int, int>>& movable_obstacles){
    // Clear the path and movable obstacles
    path.clear();
    movable_obstacles.clear();

    shared_ptr<Node> node = current_node;
    while (node != nullptr) {
        path.push_back(node->location);
        // Add movable obstacles in the path
        if(obstacle_map[node->location.first][node->location.second] == 1){
            movable_obstacles.push_back(node->location);
        }
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
}

/**
 * @brief Find the A* path
 * 
 * @param obstacle_map 
 * @param start 
 * @param goal 
 * @param heuristic_map 
 * @param agent_id 
 * @param constraints 
 * @param path 
 */
void AStar::findAStarPath(const Map& obstacle_map, const pair<int, int>& start, const pair<int, int>& goal, 
                            const Map& heuristic_map, int agent_id, const AgentType& agent_type, 
                            const vector<Constraint>& constraints, vector<pair<int, int>>& path, 
                            vector<pair<int, int>>& movable_obstacles, int starting_time_step){
    std::cout << "Finding A* path" << endl;

    // Build constraint table
    ConstraintTable constraint_table;
    buildConstraintTable(constraints, agent_id, constraint_table);

    // Earliest goal time step
    int earliest_goal_time_step = 0;
    for (auto constraint:constraints) {
        if (constraint.agent_id == agent_id && constraint.location[0] == goal) {
            if(constraint.time_step > earliest_goal_time_step){
                earliest_goal_time_step = constraint.time_step;
            }
        }
    }

    // Initialize open list
    priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, CompareNodes> open_list;
    shared_ptr<Node> root_node = make_shared<Node>(start, 0, heuristic_map[start.first][start.second], nullptr, 0);
    open_list.push(root_node);

    // Initialize closed list
    unordered_map<int, shared_ptr<Node>> closed_list;
    int x_size = obstacle_map.size();
    int y_size = obstacle_map[0].size();

    // Initialize g values
    vector<vector<float>> g_values(x_size, vector<float>(y_size, INT32_MAX));
    g_values[start.first][start.second] = 0;

    int max_time_step = 120;

    // Start A* search
    while (!open_list.empty()) {
        shared_ptr<Node> current_node = open_list.top();
        open_list.pop();

        // Check if goal is reached
        if (current_node->location == goal && current_node->time_step >= earliest_goal_time_step) {
            std::cout << "Goal reached" << endl;
            getPath(obstacle_map, agent_type, current_node, path, movable_obstacles);
            return;
        }

        // Check if time step exceeds maximum time step
        if (current_node->time_step > max_time_step) {
            std::cout << "Time step exceeded" << endl;
            return;
        }

        // Check for valid moves
        for (auto move:valid_moves_2d) {
            pair<int, int> child_location = {current_node->location.first + move.first, current_node->location.second + move.second};

            // Check if new location is within bounds or is an immovable obstacle
            if (!inMap(child_location, x_size, y_size) || obstacle_map[child_location.first][child_location.second] == -1) {
                continue;
            }

            // Additionally, for helper agents, check if the new location is a movable obstacle unless it is the start/goal
            if(agent_type == AgentType::HELPER && child_location != start && child_location != goal && obstacle_map[child_location.first][child_location.second] == 1){
                continue;
            }

            // Check if new location is in constraint table
            if (isConstrained(current_node->location, child_location, current_node->time_step + 1, constraint_table)) {
                continue;
            }

            shared_ptr<Node> child_node = make_shared<Node>(child_location, current_node->g_value + 1, heuristic_map[child_location.first][child_location.second], current_node, current_node->time_step + 1);
            
            // Check if child node is in closed list
            if (closed_list.find(GETMAPINDEX(child_location.first, child_location.second, child_node->time_step, x_size, y_size)) != closed_list.end()) {
                shared_ptr<Node> closed_node = closed_list[GETMAPINDEX(child_location.first, child_location.second, child_node->time_step, x_size, y_size)];
                if (child_node->f_value() < closed_node->f_value()) {
                    closed_list[GETMAPINDEX(child_location.first, child_location.second, child_node->time_step, x_size, y_size)] = child_node;
                    open_list.push(child_node);
                }
            }
            else {
                closed_list[GETMAPINDEX(child_location.first, child_location.second, child_node->time_step, x_size, y_size)] = child_node;
                open_list.push(child_node);
            }
        }
    }

    std::cout << "No path found" << endl;
    return;    
}