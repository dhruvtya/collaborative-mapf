#ifndef CCBS_HPP
#define CCBS_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <set>

#include "utils.hpp"
#include "heuristics.hpp"
#include "a_star.hpp"

using namespace std;
using namespace std::chrono;

/**
 * @brief Class to handle the Combined Conflict-Based Search
 */
class CCBS
{
    private:
        // Variables
        vector<vector<int>> map_;
        vector<pair<int, int>> starts_;
        vector<pair<int, int>> goals_;
        vector<pair<int, int>> helper_parkings_;

        int num_transit_agents_;
        int num_helper_agents_;
        vector<Agent> agents_list_;
        float time_horizon_ = 120;

        // Functions
        /**
         * @brief Print the agents list
        */
        void printAgentsList();

        /**
         * @brief Detect the first collision for a pair of agents
         * 
         * @param path1 The path of the first agent
         * @param path2 The path of the second agent
         * @param agent1 The ID of the first agent
         * @param agent2 The ID of the second agent
         * 
         * @return The collision detected
        */
        Collision detectFirstCollisionForPair(const vector<pair<int, int>> &path1, const vector<pair<int, int>> &path2, int agent1, int agent2);

        /**
         * @brief Detect collisions for all pairs of agents
         * 
         * @param paths The paths of all agents
         * @param collisions Reference for the collisions detected to be returned
        */
        void detectCollisions(const vector<vector<pair<int, int>>> &paths, const vector<vector<pair<int, int>>> &helper_paths, vector<Collision> &collisions);

        /**
         * @brief Generate constraints for the collision
         * 
         * @param collision The collision detected
         * 
         * @return The constraints generated
        */
        vector<Constraint> generateConstraints(const Collision &collision);

        /**
         * @brief Generate Movable Obstacle constraints for helper agents
         * 
         * @param helper_agents_list The list of helper agents
         * 
         * @return The constraints generated
         * 
        */
        vector<Constraint> generateHelperMOConstraints(const vector<Agent> &helper_agents_list);

        /**
         * @brief Generate Movable Obstacle constraints for transit agents
         * 
         * @param helper_results The results of the helper agents
         * 
         * @return The constraints generated
         * 
         * @note The helper agents should be solved before calling this function
        */
        vector<Constraint> generateTransitMOConstraints(const vector<Result> &helper_results);

    public:
        /**
         * @brief Constructer for CCBS class
         * 
         * @param map The map of the environment
         * @param starts The starting locations of the agents
         * @param goals The goal locations of the agents
         * @param helper_parkings The helper agents parkings
        */
        CCBS(vector<vector<int>> map, vector<pair<int, int>> starts, vector<pair<int, int>> goals, vector<pair<int, int>> helper_parkings);

        /**
         * @brief Destructor for CCBS class
        */
        ~CCBS(){}
        
        /**
         * @brief Solve the CCBS problem
         * 
         * @return The results of the CCBS solver
        */
        vector<Result> solve();
};


#endif // CCBS_HPP