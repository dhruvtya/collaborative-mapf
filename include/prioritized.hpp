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


/**
 * @brief Class to represent an Agent
*/
class Agent{
    public:
        // Variables
        int id_;
        AgentType type_;
        pair<int, int> start_;
        pair<int, int> goal_;
        vector<vector<int>> heuristics_;

        /**
         * @brief Overload the < operator for the priority queue
         * 
         * @param other Reference to the other agent being compared with
         * @return True if the agent is less than the other agent, False otherwise
        */
        bool operator<(const Agent& other) const;

        /**
         * @brief Construct a new Agent object
         * 
         * @param id Agent ID
         * @param type Agent Type
         * @param start Agent Start Location
         * @param goal Agent Goal Location
         * @param heuristics Agent Heuristics
        */
        Agent(int id, AgentType type, pair<int, int> start, pair<int, int> goal, vector<vector<int>> heuristics);

        /**
         * @brief Overload the == operator for the priority queue
         * 
         * @param other Reference to the other agent being compared with
         * @return True if the agents are equal, False otherwise
        */
        bool operator==(const Agent& other) const;
};

/**
 * @brief Class to handle the Prioritized Planning
*/
class PrioritizedPlanning{
    private:
        // Variables
        vector<vector<int>> map_;
        vector<pair<int, int>> starts_;
        vector<pair<int, int>> goals_;
        vector<pair<int, int>> helper_parkings_;
        
        int num_transit_agents_;
        int num_helper_agents_;
        priority_queue<Agent> agents_queue_;
        vector<Agent> solved_agents_;
        int time_horizon_ = 120;

        /**
         * @brief Function to print the priority queue
        */
        void printPriorityQueue();

        /**
         * @brief Function to check if the agent is allowed through the movable obstacle
         * 
         * @param agent Agent to check
         * @param path Path of the agent
         * @param constraints Constraints
         * @param movable_obstacles Movable Obstacles
         * @return True if the agent is allowed through the movable obstacle, False otherwise
        */
        bool isAgentAllowedThroughMO(const Agent &agent, const vector<pair<int, int>> &path, const vector<Constraint> &constraints, const vector<pair<int, int>> &movable_obstacles);

    public:
        /**
         * @brief Construct a new Prioritized Planning object
         * 
         * @param map Map of the environment
         * @param starts Start locations of the agents
         * @param goals Goal locations of the agents
         * @param helper_parkings Helper parkings
        */
        PrioritizedPlanning(vector<vector<int>> map, vector<pair<int, int>> starts, vector<pair<int, int>> goals, vector<pair<int, int>> helper_parkings);

        /**
         * @brief Destroy the Prioritized Planning object
        */
        ~PrioritizedPlanning(){};
    
        /**
         * @brief Function to solve the prioritized planning
         * 
         * @return Vector of results
        */
        vector<Result> solve();
};

#endif // PRIORITIZED_HPP