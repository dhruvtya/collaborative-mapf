#include "ccbs.hpp"

CCBS::CCBS(vector<vector<int>> map, vector<pair<int, int>> starts, vector<pair<int, int>> goals, vector<pair<int, int>> helper_parkings)
                                                        : map_{map}, starts_{starts}, goals_{goals}, helper_parkings_{helper_parkings}{
    // Initialize variables
    num_transit_agents_ = goals_.size();
    agents_list_ = vector<Agent>();

    // Create agents
    for(int i = 0; i < num_transit_agents_; i++){
        vector<vector<int>> heuristics;
        computeHeuristics(map_, goals_[i], heuristics);
        agents_list_.emplace_back(Agent(i, AgentType::TRANSIT, starts_[i], goals_[i], heuristics));
    }

    // Print agents
    printAgentsList();
}

void CCBS::printAgentsList(){
    cout << "Agents list" << endl;
    for(auto &agent : agents_list_){
        cout << "Agent ID: " << agent.id_ << ", Type: " << ((int)agent.type_?"Helper":"Transit") << 
                ", Start: " << agent.start_.first << " " << agent.start_.second << 
                ", Goal: " << agent.goal_.first << " " << agent.goal_.second << endl;
    }
}

Collision CCBS::detectFirstCollisionForPair(const vector<pair<int, int>> &path1, const vector<pair<int, int>> &path2, int agent1, int agent2, int start1, int start2){
    
    int end1 = start1 + path1.size();
    int end2 = start2 + path2.size();

    // If either of the paths is empty, return empty collision
    if(path1.size() == 0 || path2.size() == 0){
        return Collision();
    }

    int earliest_start_time = (start1 < start2) ? start1 : start2;
    int latest_end_time = (end1 > end2) ? end1 : end2;

    for (int t = earliest_start_time; t <= latest_end_time; t++) {
        int path1_ind = t - start1;
        int path2_ind = t - start2;

        int path1_next_ind = path1_ind + 1;
        int path2_next_ind = path2_ind + 1;

        if (path1_ind < 0 || path2_ind < 0) {
            continue;
        }

        if (path1_ind >= path1.size()) {
            path1_ind = path1.size() - 1;
        }

        if (path2_ind >= path2.size()) {
            path2_ind = path2.size() - 1;
        }

        if(path1[path1_ind] == path2[path2_ind]){
            return Collision{agent1, agent2, {path1[path1_ind]}, t};
        }

        if (path1_next_ind >= path1.size() && path2_next_ind >= path2.size()) {
            continue;
        }

        if(path1[path1_ind] == path2[path2_next_ind] && path1[path1_next_ind] == path2[path2_ind]){
            return Collision{agent1, agent2, {path1[path1_next_ind], path1[path1_ind]}, t};
        }

    }

    return Collision();

}

Collision CCBS::detectFirstCollisionForHelperPath(const vector<pair<int, int>> &path, const vector<pair<int, int>> &path_helper, int agent, int start){
    
    int end = start + path.size();
    int helper_end = path_helper.size() - 2;

    // If either of the paths is empty, return empty collision
    if(path.size() == 0 || path_helper.size() == 0){
        return Collision();
    }

    for (int t = 0; t <= helper_end; t++) {
        int path_ind = t - start;
        int path_next_ind = path_ind + 1;

        if (path_ind < 0 || path_ind >= path.size()) {
            continue;
        }

        if(path[path_ind] == path_helper[t]){
            return Collision{agent, -1, {path[path_ind]}, t};
        }

        if (path_next_ind >= path.size() || (t+1) >= path_helper.size()) {
            continue;
        }

        if(path[path_ind] == path_helper[t+1] && path[path_next_ind] == path_helper[t+1]){
            return Collision{agent, -1, {path[path_next_ind], path[path_ind]}, t};
        }

    }

    return Collision();

}

void CCBS::detectCollisions(const vector<vector<pair<int, int>>> &paths, const vector<vector<pair<int, int>>> &helper_paths, vector<Collision> &collisions, const vector<int>& start_times){
    // Detect collisions for all pairs of agents
    for(int i = 0; i < paths.size(); i++){
        for(int j = i + 1; j < paths.size(); j++){
            Collision collision = detectFirstCollisionForPair(paths[i], paths[j], i, j, start_times[i], start_times[j]);
            if(collision.loc.size() > 0){
                collisions.push_back(collision);
            }
        }

        // Check for collisions with initial paths of helper agents
        for (int j = 0; j < helper_paths.size(); j++) {
            Collision collision = detectFirstCollisionForHelperPath(paths[i], helper_paths[j], i, start_times[i]);
            if(collision.loc.size() > 0){
                collisions.push_back(collision);
            }
        }
    }
}

vector<Constraint> CCBS::generateConstraints(const Collision &collision){

    if(collision.agent2 != -1){
        // If the collision is a vertex collision
        if(collision.loc.size() == 1){
            return {Constraint{collision.agent1, {collision.loc[0]}, collision.timestep, false}, 
                    Constraint{collision.agent2, {collision.loc[0]}, collision.timestep, false}};
        }

        // If the collision is an edge collision
        return {Constraint{collision.agent1, {collision.loc[1], collision.loc[0]}, collision.timestep, false}, 
                Constraint{collision.agent2, {collision.loc[0], collision.loc[1]}, collision.timestep, false}};
    }

    // If one of the agents is helper agent
    if(collision.loc.size() == 1){
        return {Constraint{collision.agent1, {collision.loc[0]}, collision.timestep, false}};
    }
    return {Constraint{collision.agent1, {collision.loc[1], collision.loc[0]}, collision.timestep, false}};
    
}

vector<Constraint> CCBS::generateHelperMOConstraints(const vector<Agent> &helper_agents_list){
    // A helper agent can not cross another helper agent's assigned movable obstacle for all time steps
    vector<Constraint> constraints;
    for(int i = 0; i < helper_agents_list.size(); i++){
        for(int j = 0; j < helper_agents_list.size(); j++){
            if(i != j){
                for(int t = 0; t < time_horizon_; t++){
                    constraints.push_back(Constraint{helper_agents_list[i].id_, {helper_agents_list[j].goal_}, t, false});
                }
            }
        }
    }

    return constraints;
}

vector<Constraint> CCBS::generateTransitMOConstraints(const vector<Result> &helper_results){

    vector<vector<pair<int, int>>> paths;
    for(const auto &hr:helper_results){
        paths.push_back(hr.path_);
    }

    // Get list of all movable obstacles in the map
    set<pair<int, int>> movable_obstacles;
    for(int i = 0; i < map_.size(); i++){
        for(int j = 0; j < map_[0].size(); j++){
            if(map_[i][j] == 1){
                movable_obstacles.insert({i, j});
            }
        }
    }

    // For each helper agent find the movable obstacle in its path
    vector<Constraint> constraints;
    vector<pair<int, int>> moved_obstacles;
    vector<int> moved_obstacles_timesteps;
    for(int i = 0; i < paths.size(); i++){
        set<pair<int, int>> temp_obstacles = movable_obstacles;
        for(auto &mo:temp_obstacles){
            auto it = find(paths[i].begin(), paths[i].end(), mo);
            if(it != paths[i].end()){
                moved_obstacles.push_back(mo);
                moved_obstacles_timesteps.push_back((int)(it - paths[i].begin()));
                movable_obstacles.erase(mo);
            }
        }
    }


    // Find longest path
    int longest_path = 0;
    for(const auto& path:paths){
        if(path.size() > longest_path){
            longest_path = path.size();
        }
    }

    // Add constraints for each moved obstacle
    for(int i = 0; i < moved_obstacles.size(); i++){
        // For each agent
        for(int j = 0; j < num_transit_agents_; j++){
            // For each timestep till obstacle is moved
            for(int k = 0; k < moved_obstacles_timesteps[i]; k++){
                constraints.push_back(Constraint{j, {moved_obstacles[i]}, k, true});
            }
        }
    }

    // Add constraints across all timesteps for each unmoved obstacle
    for(const auto &mo:movable_obstacles){
        for(int i = 0; i < num_transit_agents_; i++){
            for(int j = 0; j < longest_path; j++){
                constraints.push_back(Constraint{i, {mo}, j, true});
            }
        }
    }

    return constraints;       
}

vector<Result> CCBS::solve(){
    cout << "Solving CCBS planning" << endl;
    
    vector<Result> results;
    long long unsigned int visited_nodes = 0;

    // Start timer
    auto start_time = high_resolution_clock::now();

    // Find movable objects in the paths of the agents
    set<int> helpers_used;
    set<int> helpers_unused;
    vector<Agent> helper_agents_list;
    set<pair<int, int>> movable_obstacles_found;
    for(int i = 0; i < helper_parkings_.size(); i++){
        helpers_unused.insert(i);
    }
    for(auto &agent : agents_list_){
        vector<pair<int, int>> path;
        vector<pair<int, int>> movable_obstacles;
        AStar::findAStarPath(map_, agent.start_, agent.goal_, agent.heuristics_, agent.id_, agent.type_, vector<Constraint>(), path, movable_obstacles, agent.start_time_);
        if(path.empty()){
            cout << "\nNo path found for agent " << agent.id_ << endl;
            return vector<Result>();
        }
        movable_obstacles_found.insert(movable_obstacles.begin(), movable_obstacles.end());
    }
    // Assign the closest helper for each movable obstacle
    for(auto &obstacle : movable_obstacles_found){
        int min_distance = numeric_limits<int>::max();
        int helper_id = -1;

        for(auto &helper : helpers_unused){
            if(utils::getManhattanDistance(helper_parkings_[helper], obstacle) < min_distance){
                min_distance = utils::getManhattanDistance(helper_parkings_[helper], obstacle);
                helper_id = helper;
            }
        }

        if(helper_id != -1){
            helpers_used.insert(helper_id);
            helpers_unused.erase(helper_id);
        }
        else{
            cout << "Not enough helpers" << endl;
            return results;
        }
        
        vector<vector<int>> heuristics;
        computeHeuristics(map_, helper_parkings_[helper_id], heuristics);
        helper_agents_list.emplace_back(num_transit_agents_ + helpers_used.size() - 1, AgentType::HELPER, helper_parkings_[helper_id], obstacle, heuristics);
    }

    num_helper_agents_ = helper_agents_list.size();

    // First, solve CBS for the helper agents
    // Initialize the constraint tree search
    vector<Result> helper_results;
    vector<vector<pair<int, int>>> helper_paths;

    if(helper_agents_list.size() > 0){
        priority_queue<shared_ptr<CTNode>, vector<shared_ptr<CTNode>>, CompareCTNode> open_list;
        shared_ptr<CTNode> root {new CTNode {0, 
                                            generateHelperMOConstraints(helper_agents_list), 
                                            vector<vector<pair<int, int>>>(num_transit_agents_ + helper_agents_list.size()), 
                                            vector<Collision>()}};

        
        // Find initial paths for each agent
        for(const auto &agent : helper_agents_list){
            vector<pair<int, int>> path;
            vector<pair<int, int>> movable_obstacles;

            // Find path to goal and back to parking
            vector<pair<int, int>> path;
            AStar::findAStarPath(map_, agent.start_, agent.goal_, agent.heuristics_, agent.id_, agent.type_, root->constraints, path, movable_obstacles, agent.start_time_);
            if(path.empty()){
                cout << "\nNo path found for agent " << agent.id_ << endl;
                return vector<Result>();
            }

            root->paths[agent.id_] = path;
        }

        // Get the sum of costs
        root->cost = utils::getSumOfCosts(root->paths);
        // Detect collisions
        detectCollisions(root->paths, vector<vector<pair<int, int>>>(), root->collisions, vector<int>(num_helper_agents_, 0));
        
        // Add the root to the open list
        open_list.push(root);

        // Start the search
        cout << "Solving CBS for helper agents" << endl;
        while(!open_list.empty()){
            // Get the node with the lowest cost
            shared_ptr<CTNode> current_node = open_list.top();
            open_list.pop();
            visited_nodes++;

            cout << "\rVisited helper nodes: " << visited_nodes << "/" << visited_nodes + open_list.size() << "\t" << flush;

            // Check if the current node has any collisions
            if(current_node->collisions.size() == 0){
                // If there are no collisions, return the paths
                for(int i = num_transit_agents_; i < current_node->paths.size(); i++){
                    helper_paths.push_back(current_node->paths[i]);
                    helper_results.emplace_back(i, helper_agents_list[i - num_transit_agents_].type_, helper_agents_list[i - num_transit_agents_].start_, helper_agents_list[i - num_transit_agents_].goal_, current_node->paths[i]);
                }
                break;
            }

            // Generate constraints based on a collision
            vector<Constraint> constraints = generateConstraints(current_node->collisions[0]);

            // Create two new nodes with the constraints
            for(auto &constraint:constraints){
                shared_ptr<CTNode> child_node {new CTNode {0, 
                                                            current_node->constraints, 
                                                            current_node->paths, 
                                                            vector<Collision>()}};
                child_node->constraints.push_back(constraint);

                // Plan the path for the agent with the constraint
                vector<pair<int, int>> path;
                vector<pair<int, int>> movable_obstacles;

                AStar::findAStarPath(map_, helper_agents_list[constraint.agent_id - num_transit_agents_].start_, 
                                            helper_agents_list[constraint.agent_id - num_transit_agents_].goal_, 
                                            helper_agents_list[constraint.agent_id - num_transit_agents_].heuristics_, 
                                            helper_agents_list[constraint.agent_id - num_transit_agents_].id_, 
                                            helper_agents_list[constraint.agent_id - num_transit_agents_].type_, 
                                            child_node->constraints, path, movable_obstacles, helper_agents_list[constraint.agent_id - num_transit_agents_].start_time_);
                if(path.empty()){
                    continue;
                }

                // Update the path for the agent
                child_node->paths[constraint.agent_id] = path;

                // Get the sum of costs
                child_node->cost = utils::getSumOfCosts(child_node->paths);
                // Detect collisions
                detectCollisions(child_node->paths, vector<vector<pair<int, int>>>(), child_node->collisions, vector<int>(num_helper_agents_, 0));


                // Add the child to the open list
                open_list.push(child_node);
            }
        }

        if(helper_results.size() == 0){
            cout << "No solution found for helper agents" << endl;
            return helper_results;
        }
    }

    num_helper_agents_ = helper_results.size();
    vector <int> start_times(num_transit_agents_, 0);

    // Add helper agents to the transit agent list, converting them to transit
    for(int i = 0; i < helper_agents_list.size(); i++){
        vector<vector<int>> heuristics;
        computeHeuristics(map_, helper_agents_list[i].start_, heuristics);
        int start_time = helper_results[i].path_.size() - 1;
        start_times.push_back(start_time);
        int id = helper_results[i].agent_id_;
        agents_list_.emplace_back(Agent(id, AgentType::CONVERTED_TRANSIT, helper_agents_list[i].goal_, helper_agents_list[i].start_, heuristics, start_time));
    }

    // Solve CBS for all agents
    // Initialize the constraint tree search
    visited_nodes = 0;

    priority_queue<shared_ptr<CTNode>, vector<shared_ptr<CTNode>>, CompareCTNode> open_list;
    shared_ptr<CTNode> root {new CTNode {0, 
                                        generateTransitMOConstraints(helper_results), 
                                        vector<vector<pair<int, int>>>(agents_list_.size()), 
                                        vector<Collision>()}};
    
    // Find initial paths for each agent
    for(const auto &agent : agents_list_){
        vector<pair<int, int>> path;
        vector<pair<int, int>> movable_obstacles;
        AStar::findAStarPath(map_, agent.start_, agent.goal_, agent.heuristics_, agent.id_, agent.type_, root->constraints, path, movable_obstacles, agent.start_time_);
        if(path.empty()){
            cout << "\nNo path found for agent " << agent.id_ << endl;
            return vector<Result>();
        }
        root->paths[agent.id_] = path;
    }

    // Get the sum of costs
    root->cost = utils::getSumOfCosts(root->paths);
    // Detect collisions
    vector<vector<pair<int, int>>> collision_checking_paths = root->paths;
    detectCollisions(collision_checking_paths, helper_paths, root->collisions, start_times);
    // Add the root to the open list
    open_list.push(root);

    // Start the search
    cout << "Solving CBS for transit agents" << endl;
    while(!open_list.empty()){
        // Get the node with the lowest cost
        if (open_list.empty()) {
            break;
        }
        shared_ptr<CTNode> current_node = open_list.top();
        open_list.pop();
        visited_nodes++;

        // Check if the current node has any collisions
        if(current_node->collisions.size() == 0){
            // If there are no collisions, return the paths. First, append transit agent paths
            for(int i = 0; i < num_transit_agents_; i++){
                results.emplace_back(i, agents_list_[i].type_, agents_list_[i].start_, agents_list_[i].goal_, current_node->paths[i]);
            }

            //Then, append results of agents converted from helper to transit
            for (int i = 0; i < num_helper_agents_; i++) {
                helper_results[i].path_.insert(helper_results[i].path_.end(), current_node->paths[i + num_transit_agents_].begin() + 1, current_node->paths[i + num_transit_agents_].end());
            }

            // Add helper results
            results.insert(results.end(), helper_results.begin(), helper_results.end());
            break;
        }

        // Generate constraints based on a collision
        vector<Constraint> constraints = generateConstraints(current_node->collisions[0]);

        // Create two new nodes with the constraints
        for(auto &constraint:constraints){
            shared_ptr<CTNode> child_node {new CTNode {0, 
                                                        current_node->constraints, 
                                                        current_node->paths, 
                                                        vector<Collision>()}};

            child_node->constraints.push_back(constraint);
            
            // Find the path for the agent with the constraint
            vector<pair<int, int>> path;
            vector<pair<int, int>> movable_obstacles;
            AStar::findAStarPath(map_, agents_list_[constraint.agent_id].start_, 
                                        agents_list_[constraint.agent_id].goal_, 
                                        agents_list_[constraint.agent_id].heuristics_, 
                                        agents_list_[constraint.agent_id].id_, 
                                        agents_list_[constraint.agent_id].type_, 
                                        child_node->constraints, path, movable_obstacles, agents_list_[constraint.agent_id].start_time_);

            if(path.size() == 0){
                continue;
            }

            child_node->paths[constraint.agent_id] = path;

            // Detect collisions
            collision_checking_paths = child_node->paths;
            detectCollisions(collision_checking_paths, helper_paths, child_node->collisions, start_times);

            // Get the sum of costs
            child_node->cost = utils::getSumOfCosts(child_node->paths, map_) + child_node->collisions.size();

            // Add child to the open list
            open_list.push(child_node);
        }
    }


    // End timer
    auto end_time = high_resolution_clock::now();
    auto computation_time = duration_cast<milliseconds>(end_time - start_time);
    
    if(results.size() == 0){
        cout << "No solution found" << endl;
        return results;
    }

    // Metrics
    cout << "\nFound solution ----------" << endl;
    cout << "| Comp. time: " << duration_cast<milliseconds>(end_time - start_time).count() << "ms\t|" << endl;
    cout << "| Sum of costs: " << utils::getSumOfCosts(results) << "\t|" << endl;
    cout << "-------------------------" << endl;

    return results;
}

