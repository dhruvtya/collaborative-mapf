#include "prioritized.hpp"

Agent::Agent(int id, AgentType type, pair<int, int> start, pair<int, int> goal, vector<vector<int>> heuristics)
                            : id_{id}, type_{type}, start_{start}, goal_{goal}, heuristics_{heuristics}{}

bool Agent::operator<(const Agent& other) const{
    return this->id_ > other.id_;
}

bool Agent::operator==(const Agent& other) const{
    return (this->type_ == other.type_ && this->start_ == other.start_ && this->goal_ == other.goal_);
}

PrioritizedPlanning::PrioritizedPlanning(vector<vector<int>> map, 
                                        vector<pair<int, int>> starts, 
                                        vector<pair<int, int>> goals, pair<int, int> helper_parking)
                                        : map_{map}, starts_{starts}, goals_{goals}, helper_parking_{helper_parking}{
    
    num_transit_agents_ = goals_.size();
    agents_queue_ = priority_queue<Agent>();

    // Build prioritiy queue & Compute heuristics
    for(int i = 0; i < num_transit_agents_; i++){
        // TODO : Compute heuristics
        vector<vector<int>> heuristics;
        computeHeuristics(map_, goals_[i], heuristics);
        agents_queue_.emplace(10 * i + 10, AgentType::TRANSIT, starts_[i], goals_[i], heuristics);
    }

    // Print priority queue
    printPriorityQueue();
}

void PrioritizedPlanning::printPriorityQueue(){
    cout << "Printing priority queue" << endl;
    priority_queue<Agent> temp_queue = agents_queue_;
    while(!temp_queue.empty()){
        Agent agent = temp_queue.top();
        cout << "Agent ID: " << agent.id_ << ", Type: " << ((int)agent.type_?"Helper":"Transit") << 
                ", Start: " << agent.start_.first << " " << agent.start_.second << 
                ", Goal: " << agent.goal_.first << " " << agent.goal_.second << endl;
        temp_queue.pop();
    }
}

vector<Result> PrioritizedPlanning::solve(){
    cout << "Solving prioritized planning" << endl;
    solved_agents_ = queue<Agent>();
    vector<Constraint> constraints;
    vector<Result> results;

    // Start timer
    auto start_time = high_resolution_clock::now();

    // Loop to find path for each agent
    while(!agents_queue_.empty()){
        // Get agent with highest priority
        Agent agent = agents_queue_.top();

        // Find path for agent
        vector<pair<int, int>> path;
        vector<pair<int, int>> movable_obstacles;

        if(agent.type_ == AgentType::TRANSIT){
            AStar::findAStarPath(map_, agent.start_, agent.goal_, agent.heuristics_, agent.id_, agent.type_, constraints, path, movable_obstacles);
        }
        else{
            // If agent is helper, find path to goal and back to parking
            vector<pair<int, int>> path1, path2;
            AStar::findAStarPath(map_, agent.start_, agent.goal_, agent.heuristics_, agent.id_, agent.type_, constraints, path1, movable_obstacles);
            AStar::findAStarPath(map_, agent.goal_, agent.start_, agent.heuristics_, agent.id_, agent.type_, constraints, path2, movable_obstacles);
            path = path1;
            path.insert(path.end(), path2.begin(), path2.end());
        }

        // If a path is found, check if there are any movable obstacles
        if(!path.empty()){
            // If there are no movable obstacles in the path
            if(movable_obstacles.empty() || agent.type_ == AgentType::HELPER){
                // Remove agent from queue and add to solved agents
                // Add results
                results.emplace_back(Result{agent.id_, agent.type_, agent.start_, agent.goal_, path});
                solved_agents_.push(agent);
                agents_queue_.pop();

                // Add constraints for lower priority agents
                priority_queue<Agent> temp_queue = agents_queue_;
                while(!temp_queue.empty()){
                    Agent temp_agent = temp_queue.top();
                    temp_queue.pop();
                    
                    for(int i = 0; i < path.size(); i++){
                        vector<pair<int, int>> temp_path;
                        temp_path.push_back(path[i]);
                        constraints.emplace_back(Constraint{temp_agent.id_, temp_path, i});
                    }
                    for(int i = 0; i < path.size() - 1; i++){
                        vector<pair<int, int>> temp_path;
                        temp_path.push_back(path[i]);
                        temp_path.push_back(path[i+1]);
                        constraints.emplace_back(Constraint{temp_agent.id_, temp_path, i + 1});
                    }
                    if(path.size() < time_horizon_){
                        for(int i = path.size(); i < time_horizon_; i++){
                            vector<pair<int, int>> temp_path;
                            temp_path.push_back(path.back());
                            constraints.emplace_back(Constraint{temp_agent.id_, temp_path, i});
                        }
                    }
                }
            }
            else{
                // If there are movable obstacles in the path of a transit agent, add higher priority helper agents
                // Can only accomodate 9 helper agents per transit agent
                if(movable_obstacles.size() > 9){
                    cout << "Too many movable obstacles in the path of agent " << agent.id_ << endl;
                    return vector<Result>();
                }
                else{
                    for(int i = 0; i < movable_obstacles.size(); i++){
                        vector<vector<int>> heuristics;
                        computeHeuristics(map_, movable_obstacles[i], heuristics);
                        agents_queue_.emplace(agent.id_ - 9 + i, AgentType::HELPER, helper_parking_, movable_obstacles[i], heuristics);
                    }
                }
            }
        }
        else{
            // If no path is found, return failure
            cout << "No path found for agent " << agent.id_ << endl;
            return vector<Result>();
        }
    }

    // End timer
    auto end_time = high_resolution_clock::now();
    auto computation_time = duration_cast<milliseconds>(end_time - start_time);
    
    // Metrics
    cout << "Found solution ----------" << endl;
    cout << "| Comp. time: " << computation_time.count() / 1000 << "s\t|" << endl;
    cout << "| Sum of costs: " /* TODO */ << "\t|" << endl;
    cout << "-------------------------" << endl;

    // Print result
    utils::printResults(results);
    
    return results;
}