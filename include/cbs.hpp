#ifndef CBS_HPP
#define CBS_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <set>

#include "utils.hpp"
#include "heuristics.hpp"
#include "a_star.hpp"

using namespace std;
using namespace std::chrono;

struct Collision{
    int agent1;
    int agent2;
    vector<pair<int, int>> loc;
    int timestep;
};

struct CTNode{
    double cost;
    vector<Constraint> constraints;
    vector<vector<pair<int, int>>> paths;
    vector<Collision> collisions;
};

struct CompareCTNode{
    bool operator()(const shared_ptr<CTNode> &lhs, const shared_ptr<CTNode> &rhs) const{
        return lhs->cost > rhs->cost;
    }
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
        Collision detectFirstCollisionForPair(const vector<pair<int, int>> &path1, const vector<pair<int, int>> &path2, int agent1, int agent2);
        void detectCollisions(const vector<vector<pair<int, int>>> &paths, vector<Collision> &collisions);
        vector<Constraint> generateConstraints(const Collision &collision);

    public:
        CBS(vector<vector<int>> map, vector<pair<int, int>> starts, vector<pair<int, int>> goals, vector<pair<int, int>> helper_parkings);
        ~CBS(){}
        vector<Result> solve();
};


#endif // CBS_HPP