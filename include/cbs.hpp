#ifndef CBS_HPP
#define CBS_HPP

#include <iostream>
#include <chrono>
#include <vector>

#include "utils.hpp"
#include "heuristics.hpp"
#include "a_star.hpp"

using namespace std;
using namespace std::chrono;

struct CTNode{
    // TODO

};

/**
 * @brief Class to handle the Conflict-Based Search
 */
class CBS
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

        void printAgentsList();

    public:
        CBS(vector<vector<int>> map, vector<pair<int, int>> starts, vector<pair<int, int>> goals, vector<pair<int, int>> helper_parkings);
        ~CBS(){}
        vector<Result> solve();
};


#endif // CBS_HPP