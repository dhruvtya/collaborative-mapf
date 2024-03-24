#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "iostream"
#include "unordered_map"
#include "algorithm"
#include "heuristics.hpp"
#include "utils.hpp"

using namespace std;

// Constraints
struct Constraint{
    int agent_id;
    vector<pair<int, int>> location;
    int time_step;
};

typedef unordered_map<int, vector<Constraint>> ConstraintTable;

namespace AStar{
    struct Node {
        pair<int, int> location;
        float g_value;
        float h_value;
        shared_ptr<Node> parent;
        int time_step;

        Node(Node& node)
            : location{node.location}, g_value{node.g_value}, h_value{node.h_value}, parent{node.parent}, time_step{node.time_step}{}

        Node(pair<int, int> location, float g_value, float h_value, shared_ptr<Node> parent, int time_step)
            : location{location}, g_value{g_value}, h_value{h_value}, parent{parent}, time_step{time_step}{}

        float f_value() const{
            return g_value + h_value;
        }
    };

    struct CompareNodes
    {
        bool operator() (shared_ptr<Node> lhs, shared_ptr<Node> rhs) const
        {
            return lhs->f_value() > rhs->f_value();
        }
    };
    
    void buildConstraintTable(const vector<Constraint>& constraints, const int& agent_id, unordered_map<int, vector<Constraint>>& constraint_table);

    bool inMap(const pair<int, int>& location, const int& x_size, const int& y_size);

    bool isConstrained(const pair<int, int>& curr_location, const pair<int, int>& next_location, const int& next_time, const ConstraintTable& constraint_table);

    void getPath(const Map& obstacle_map, const AgentType& agent_type, const shared_ptr<Node>& current_node, vector<pair<int, int>>& path, vector<pair<int, int>>& movable_obstacles);

    void findAStarPath(const Map& obstacle_map, const pair<int, int>& start, const pair<int, int>& goal, const Map& heuristic_map, int agent_id, const AgentType& agent_type, const vector<Constraint>& constraints, vector<pair<int, int>>& path, vector<pair<int, int>>& movable_obstacles);
}

#endif // A_STAR_HPP