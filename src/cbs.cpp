#include "cbs.hpp"

CBS::CBS(vector<vector<int>> map, vector<pair<int, int>> starts, vector<pair<int, int>> goals, vector<pair<int, int>> helper_parkings)
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

void CBS::printAgentsList(){
    cout << "Agents list" << endl;
    for(auto &agent : agents_list_){
        cout << "Agent ID: " << agent.id_ << ", Type: " << ((int)agent.type_?"Helper":"Transit") << 
                ", Start: " << agent.start_.first << " " << agent.start_.second << 
                ", Goal: " << agent.goal_.first << " " << agent.goal_.second << endl;
    }
}

Collision CBS::detectFirstCollisionForPair(const vector<pair<int, int>> &path1, const vector<pair<int, int>> &path2, int agent1, int agent2){
    
    // If either of the paths is empty, return empty collision
    if(path1.size() == 0 || path2.size() == 0){
        return Collision();
    }

    // If path sizes are equal
    if(path1.size() == path2.size()){
        // For the path lengths
        for(int i = 0; i < path1.size(); i++){
            // Check for vertex collision
            if(path1[i] == path2[i]){
                return Collision{agent1, agent2, {path1[i]}, i};
            }
            // Check for edge collision
            if(i > 0 && path1[i] == path2[i - 1] && path1[i - 1] == path2[i]){
                return Collision{agent1, agent2, {path1[i], path1[i - 1]}, i};
            }
        }
    }
    else if(path1.size() > path2.size()){
        // For the shorter path length
        for(int i = 0; i < path2.size(); i++){
            // Check for vertex collision
            if(path1[i] == path2[i]){
                return Collision{agent1, agent2, {path1[i]}, i};
            }
            // Check for edge collision
            if(i > 0 && path1[i] == path2[i - 1] && path1[i - 1] == path2[i]){
                return Collision{agent1, agent2, {path1[i], path1[i - 1]}, i};
            }
        }
        // For the rest of the longer path, check for vertex collision at the goal
        for(int i = path2.size(); i < path1.size(); i++){
            if(path1[i] == path2.back()){
                return Collision{agent1, agent2, {path1[i]}, i};
            }
        }
    }
    else{
        // For the shorter path length
        for(int i = 0; i < path1.size(); i++){
            // Check for vertex collision
            if(path1[i] == path2[i]){
                return Collision{agent1, agent2, {path1[i]}, i};
            }
            // Check for edge collision
            if(i > 0 && path1[i] == path2[i - 1] && path1[i - 1] == path2[i]){
                return Collision{agent1, agent2, {path1[i], path1[i - 1]}, i};
            }
        }
        // For the rest of the longer path, check for vertex collision at the goal
        for(int i = path1.size(); i < path2.size(); i++){
            if(path1.back() == path2[i]){
                return Collision{agent1, agent2, {path2[i]}, i};
            }
        }
    }

    return Collision();
}

void CBS::detectCollisions(const vector<vector<pair<int, int>>> &paths, vector<Collision> &collisions){
    // Detect collisions for all pairs of agents
    for(int i = 0; i < paths.size(); i++){
        for(int j = i + 1; j < paths.size(); j++){
            Collision collision = detectFirstCollisionForPair(paths[i], paths[j], i, j);
            if(collision.loc.size() > 0){
                collisions.push_back(collision);
            }
        }
    }
}

vector<Constraint> CBS::generateConstraints(const Collision &collision){
    // If the collision is a vertex collision
    if(collision.loc.size() == 1){
        return {Constraint{collision.agent1, {collision.loc[0]}, collision.timestep, false}, 
                Constraint{collision.agent2, {collision.loc[0]}, collision.timestep, false}};
    }

    // If the collision is an edge collision
    return {Constraint{collision.agent1, {collision.loc[1], collision.loc[0]}, collision.timestep, false}, 
            Constraint{collision.agent2, {collision.loc[0], collision.loc[1]}, collision.timestep, false}};
}

vector<Result> CBS::solve(){
    cout << "Solving CBS planning" << endl;
    
    vector<Result> results;
    int helpers_used_ = 0;
    long unsigned int visited_nodes = 0;

    // Start timer
    auto start_time = high_resolution_clock::now();

    // TODO: Find movable objects in the paths of the agents

    // TODO: Assign the closest helper for each movable obstacle

    // Solve CBS for the entire fleet
    // Initialize the constraint tree search
    priority_queue<shared_ptr<CTNode>, vector<shared_ptr<CTNode>>, CompareCTNode> open_list;
    shared_ptr<CTNode> root {new CTNode {0, 
                                        vector<Constraint>(), 
                                        vector<vector<pair<int, int>>>(agents_list_.size()), 
                                        vector<Collision>()}};
    // Find initial paths for each agent
    for(const auto &agent : agents_list_){
        vector<pair<int, int>> path;
        vector<pair<int, int>> movable_obstacles;
        AStar::findAStarPath(map_, agent.start_, agent.goal_, agent.heuristics_, agent.id_, agent.type_, vector<Constraint>(), path, movable_obstacles, 0);
        root->paths[agent.id_] = path;
    }
    // Get the sum of costs
    root->cost = utils::getSumOfCosts(root->paths);
    // Detect collisions
    detectCollisions(root->paths, root->collisions);
    // Add the root to the open list
    open_list.push(root);

    // Start the search
    while(!open_list.empty()){
        // Get the node with the lowest cost
        shared_ptr<CTNode> current_node = open_list.top();
        open_list.pop();
        visited_nodes++;

        cout << "\rVisited nodes: " << visited_nodes << "/" << visited_nodes + open_list.size() << flush;

        // Check if the current node has any collisions
        if(current_node->collisions.size() == 0){
            // If there are no collisions, return the paths
            for(int i = 0; i < current_node->paths.size(); i++){
                results.emplace_back(i, agents_list_[i].type_, agents_list_[i].start_, agents_list_[i].goal_, current_node->paths[i]);
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

            // Find the path for the agent with the constraint
            vector<pair<int, int>> path;
            vector<pair<int, int>> movable_obstacles;
            AStar::findAStarPath(map_, 
                                agents_list_[constraint.agent_id].start_, 
                                agents_list_[constraint.agent_id].goal_, 
                                agents_list_[constraint.agent_id].heuristics_, 
                                constraint.agent_id, 
                                agents_list_[constraint.agent_id].type_, 
                                child_node->constraints, 
                                path, 
                                movable_obstacles, 
                                0);
            if(path.size() == 0){
                continue;
            }

            // Update the path for the agent
            child_node->paths[constraint.agent_id] = path;
            // Get the sum of costs
            child_node->cost = utils::getSumOfCosts(child_node->paths);
            // Detect collisions
            detectCollisions(child_node->paths, child_node->collisions);
            // Add the child to the open list
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

    // Print result
    // utils::printResults(results);

    return results;
}

