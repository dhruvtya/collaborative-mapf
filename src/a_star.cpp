#include "a_star.hpp"

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

void AStar::buildMOHelperConstraintTable(const vector<Constraint>& constraints, const int& agent_id, ConstraintTable& constraint_table){
    // Clear the constraint table
    constraint_table.clear();

    // Add constraints to the constraint table
    for (auto constraint:constraints) {
        if (constraint.agent_id == agent_id && constraint.for_movable_obstacle) {
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

bool AStar::inMap(const pair<int, int>& location, const int& x_size, const int& y_size){
    return (location.first >= 0 && location.first < x_size && location.second >= 0 && location.second < y_size);
}

bool AStar::isConstrained(const pair<int, int>& curr_location, const pair<int, int>& next_location, const int& next_time, const ConstraintTable& constraint_table){
    // Check if there is a constraint at the next time step
    if (constraint_table.find(next_time) != constraint_table.end()) {
        // Check if the next location is in the constraint table
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
    reverse(path.begin(), path.end());
}

void AStar::findAStarPath(const Map& obstacle_map, const pair<int, int>& start, const pair<int, int>& goal, 
                            const Map& heuristic_map, int agent_id, const AgentType& agent_type, 
                            const vector<Constraint>& constraints, vector<pair<int, int>>& path, 
                            vector<pair<int, int>>& movable_obstacles, int starting_time_step){

    // Build constraint table
    ConstraintTable constraint_table;
    buildConstraintTable(constraints, agent_id, constraint_table);
    ConstraintTable mo_helper_constraint_table;
    if(agent_type == AgentType::HELPER){
        buildMOHelperConstraintTable(constraints, agent_id, mo_helper_constraint_table);
    }

    // Earliest goal time step
    int earliest_goal_time_step = starting_time_step;
    for (auto constraint:constraints) {
        if (constraint.agent_id == agent_id && constraint.location[0] == goal) {
            if(constraint.time_step > earliest_goal_time_step){
                earliest_goal_time_step = constraint.time_step;
            }
        }
    }

    // Initialize open list
    priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, CompareNodes> open_list;
    shared_ptr<Node> root_node = make_shared<Node>(start, 0, heuristic_map[start.first][start.second], nullptr, starting_time_step);
    
    if (isConstrained(root_node->location, root_node->location, starting_time_step, constraint_table)) {
        return;
    }
    
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
            getPath(obstacle_map, agent_type, current_node, path, movable_obstacles);
            return;
        }

        // Check if time step exceeds maximum time step
        if (current_node->time_step > max_time_step) {
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
                // Check if helper agent is allowed to move through movable obstacle at this time step (has the obstacle been moved?)
                if(mo_helper_constraint_table.find(current_node->time_step) != mo_helper_constraint_table.end()){
                    bool allowed = false;
                    for(auto constraint:mo_helper_constraint_table.at(current_node->time_step)){
                        if(constraint.location[0] == child_location){
                            allowed = true;
                            break;
                        }
                    }
                    if(!allowed){
                        continue;
                    }
                }
            }

            // Check if new location is in constraint table
            if (isConstrained(current_node->location, child_location, current_node->time_step + 1, constraint_table)) {
                continue;
            }

            shared_ptr<Node> child_node = make_shared<Node>(child_location, current_node->g_value + 1, heuristic_map[child_location.first][child_location.second], current_node, current_node->time_step + 1);
            
            // If a transit agent is moving through a movable obstacle, add a large cost to the g value
            if(agent_type == AgentType::TRANSIT && obstacle_map[child_location.first][child_location.second] == 1){
                // child_node->g_value += obstacle_map.size() * obstacle_map[0].size();
                child_node->g_value += 10;
            }

            // For any agent, add a cost of 1 for moving to a new location, to avoid unnecessary moves
            if(child_location != current_node->location){
                child_node->g_value += 1;
            }
            
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

    return;    
}