#ifndef UTILS_HPP
#define UTILS_HPP

#include "iostream"
#include "vector"
#include "fstream" 
#include "memory"

using namespace std;

/**
 * @brief Enum class for the type of agent
*/
enum class AgentType{
    TRANSIT = 0,
    HELPER = 1
};

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
 * @brief Struct for the result of the search
*/
struct Result{
    int agent_id_;
    AgentType type_;
    pair<int, int> start_;
    pair<int, int> goal_;
    vector<pair<int, int>> path_;
};

/**
 * @brief Namespace for utility functions
*/
namespace utils{

    /**
     * @brief Load the map from the file
     * 
     * @param filename The name of the file
     * @param map The reference to the map to be loaded and returned
     * @param starts The reference to the starts of the agents
     * @param goals The reference to the goals of the agents
     * @param helper_parkings The reference to the helper parkings
     * 
     * @return True if the map is loaded successfully, false otherwise
    */
    bool loadMap(string &filename, vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, vector<pair<int, int>> &helper_parkings);
    
    /**
     * @brief Print the map to console
     * 
     * @param map The map to be printed
     * @param starts The starts of the agents
     * @param goals The goals of the agents
     * @param helper_parkings The helper parkings
    */
    void printMap(const vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, vector<pair<int, int>> &helper_parkings);
    
    /**
     * @brief Print heuristic map to console
     * 
     * @param map The heuristic map to be printed
    */
    void printHeuristicMap(const vector<vector<int>> &map);
    
    /**
     * @brief Print the results to console
     * 
     * @param results The results to be printed
    */
    void printResults(const vector<Result> &results);
    
    /**
     * @brief Save the solution to a csv file
     * 
     * @param results The results to be saved
     * @param filename The name of the file
    */
    void saveSolution(const vector<Result> &results, string filename);
}

#endif // UTILS_HPP
