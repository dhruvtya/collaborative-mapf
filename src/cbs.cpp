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
    cout << "Printing agents list" << endl;
    for(auto &agent : agents_list_){
        cout << "Agent ID: " << agent.id_ << ", Type: " << ((int)agent.type_?"Helper":"Transit") << 
                ", Start: " << agent.start_.first << " " << agent.start_.second << 
                ", Goal: " << agent.goal_.first << " " << agent.goal_.second << endl;
    }
}

vector<Result> CBS::solve(){
    cout << "Solving CBS planning" << endl;
    
    vector<Constraint> constraints;
    vector<Result> results;
    int helpers_used_ = 0;

    // Start timer
    auto start_time = high_resolution_clock::now();

    // Find movable objects in the paths of the agents

    // Assign the closest helper for each movable obstacle

    // Solve CBS for the entire fleet


    // End timer
    auto end_time = high_resolution_clock::now();
    auto computation_time = duration_cast<milliseconds>(end_time - start_time);
    
    // Metrics
    cout << "\nFound solution ----------" << endl;
    cout << "| Comp. time: " << duration_cast<milliseconds>(end_time - start_time).count() << "ms\t|" << endl;
    cout << "| Sum of costs: " /* TODO */ << "\t|" << endl;
    cout << "-------------------------" << endl;

    // Print result
    utils::printResults(results);

    return results;
}

