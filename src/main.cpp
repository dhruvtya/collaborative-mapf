#include "prioritized.hpp"

int main(){

    // Initialize variables
    vector<vector<int>> map;
    vector<pair<int, int>> starts;
    vector<pair<int, int>> goals;
    pair<int, int> helper_parking;
    string filename = "maps/exp_0.txt";

    // Load map
    if(!utils::loadMap(filename, map, starts, goals, helper_parking)){
        cout << "Error loading map" << endl;
        return 1;
    }

    // Print map
    utils::printMap(map, starts, goals, helper_parking);
    
    Map heuristic_map;
    computeHeuristics(map, goals[0], heuristic_map);
    utils::printHeuristicMap(heuristic_map);

    // Call prioritized planning
    PrioritizedPlanning p(map, starts, goals, helper_parking);
    p.solve();

    return 0;
}