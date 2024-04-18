#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "iostream"
#include "unordered_map"
#include "algorithm"
#include "heuristics.hpp"
#include "utils.hpp"

/**
 * @brief Macro to get the index of the map
*/
#define GETMAPINDEX(X, Y, T, XSIZE, YSIZE) (T * XSIZE * YSIZE + (Y - 1) * XSIZE + (X - 1))

using namespace std;

/**
 * @brief Typedef for the constraint table
*/
typedef unordered_map<int, vector<Constraint>> ConstraintTable;

/**
 * @brief Namespace for A* search
*/
namespace AStar{

    /**
     * @brief Struct for a node in the A* search tree
    */
    struct Node {
        pair<int, int> location;
        float g_value;
        float h_value;
        shared_ptr<Node> parent;
        int time_step;

        /**
         * @brief Overloaded constructor for the Node struct
        */
        Node(Node& node)
            : location{node.location}, g_value{node.g_value}, h_value{node.h_value}, parent{node.parent}, time_step{node.time_step}{}

        /**
         * @brief Overloaded constructor for the Node struct
        */
        Node(pair<int, int> location, float g_value, float h_value, shared_ptr<Node> parent, int time_step)
            : location{location}, g_value{g_value}, h_value{h_value}, parent{parent}, time_step{time_step}{}

        /**
         * @brief Calculate the f value of the node
         * 
         * @return The f value of the node
        */
        float f_value() const{
            return g_value + h_value;
        }
    };

    /**
     * @brief Struct for comparing nodes in the priority queue
    */
    struct CompareNodes
    {
        bool operator() (shared_ptr<Node> lhs, shared_ptr<Node> rhs) const
        {
            return lhs->f_value() > rhs->f_value();
        }
    };
    
    /**
     * @brief Build the constraint table for a given agent for A* search
     * 
     * @param constraints - List of constraints
     * @param agent_id - ID of the agent for which the constraint table is being built
     * @param constraint_table - Reference to the constraint table [Return]
     */
    void buildConstraintTable(const vector<Constraint>& constraints, const int& agent_id, unordered_map<int, vector<Constraint>>& constraint_table);

    /**
     * @brief Build the constraint table only for movable obstacles for a given agent for A* search
     * 
     * @param constraints - List of constraints
     * @param agent_id - ID of the agent for which the constraint table is being built
     * @param constraint_table - Reference to the constraint table [Return]
     */
    void buildMOHelperConstraintTable(const vector<Constraint>& constraints, const int& agent_id, unordered_map<int, vector<Constraint>>& constraint_table);

    /**
     * @brief Check if a location is within the map bounds
     * 
     * @param location The location to be checked 
     * @param x_size The x size of the map
     * @param y_size The y size of the map
     * @return true if the location is within the map bounds, false otherwise
     */
    bool inMap(const pair<int, int>& location, const int& x_size, const int& y_size);

    /**
     * @brief Check if a move is constrained by the constraint table
     * 
     * @param curr_location Current location of the agent 
     * @param next_location Next location of the agent
     * @param next_time Next time step of the agent
     * @param constraint_table The constraint table for the agent
     * @return true if the move is constrained, false otherwise
     */
    bool isConstrained(const pair<int, int>& curr_location, const pair<int, int>& next_location, const int& next_time, const ConstraintTable& constraint_table);

    /**
     * @brief Get the path from the current node
     * 
     * @param obstacle_map The map of the environment
     * @param agent_type The type of the agent
     * @param current_node The current node of the search tree
     * @param path The path of the agent to be returned
     * @param movable_obstacles The movable obstacles found in the path
     */
    void getPath(const Map& obstacle_map, const AgentType& agent_type, const shared_ptr<Node>& current_node, vector<pair<int, int>>& path, vector<pair<int, int>>& movable_obstacles);

    /**
     * @brief Find the A* path for an agent
     * 
     * @param obstacle_map The map of the environment
     * @param start The start location of the agent
     * @param goal The goal location of the agent
     * @param heuristic_map The heuristic map for the agent on the environment
     * @param agent_id The id of the agent
     * @param constraints The constraints for the agent
     * @param path The path to be returned
     * @param movable_obstacles The movable obstacles found in the path
     * @param starting_time_step The starting time step for the plan for the agent
     */
    void findAStarPath(const Map& obstacle_map, 
                        const pair<int, int>& start, 
                        const pair<int, int>& goal, 
                        const Map& heuristic_map, 
                        int agent_id, const AgentType& agent_type, 
                        const vector<Constraint>& constraints, 
                        vector<pair<int, int>>& path, 
                        vector<pair<int, int>>& movable_obstacles, 
                        int starting_time_step);
}

#endif // A_STAR_HPP