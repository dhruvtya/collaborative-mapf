#ifndef PRIORITIZED_HPP
#define PRIORITIZED_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <queue>

#include "a_star.hpp"
#include "heuristics.hpp"
#include "utils.hpp"

using namespace std;
using namespace std::chrono;

class Agent{
    public:
        // Variables
        int id_;
        AgentType type_;
        pair<int, int> start_;
        pair<int, int> goal_;
        vector<vector<int>> heuristics_;

        // Functions
        bool operator<(const Agent& other) const;
        Agent(int id, AgentType type, pair<int, int> start, pair<int, int> goal, vector<vector<int>> heuristics);
        bool operator==(const Agent& other) const;
};

class PrioritizedPlanning{
    private:
        // Variables
        vector<vector<int>> map_;
        vector<pair<int, int>> starts_;
        vector<pair<int, int>> goals_;
        pair<int, int> helper_parking_;
        
        int num_transit_agents_;
        int num_helper_agents_;
        priority_queue<Agent> agents_queue_;
        queue<Agent> solved_agents_;
        int time_horizon_ = 10;

        // Functions
        void printPriorityQueue();

    public:
        // Constructor
        PrioritizedPlanning(vector<vector<int>> map, vector<pair<int, int>> starts, vector<pair<int, int>> goals, pair<int, int> helper_parking);

        // Destructor
        ~PrioritizedPlanning(){};
    
        // Functions
        vector<Result> solve();
};

#endif // PRIORITIZED_HPP