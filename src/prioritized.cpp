#include "prioritized.hpp"

Agent::Agent(int id, AgentType type, pair<int, int> start, pair<int, int> goal)
                            : id_{id}, type_{type}, start_{start}, goal_{goal}{}

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

    // Build prioritiy queue & Compute heuristics
    for(int i = 0; i < num_transit_agents_; i++){
        // TODO : Compute heuristics
        agents_queue_.emplace(10 * i + 10, AgentType::TRANSIT, starts_[i], goals_[i]);
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

void PrioritizedPlanning::solve(){
    cout << "Solving prioritized planning" << endl;

    // Test adding helper agent
    agents_queue_.emplace(1, AgentType::HELPER, helper_parking_, helper_parking_);

    // Print priority queue
    printPriorityQueue();
}