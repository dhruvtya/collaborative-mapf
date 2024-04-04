#include "prioritized.hpp"

int main(int argc, char* argv[]){

    // Initialize variables
    vector<vector<int>> map;
    vector<pair<int, int>> starts;
    vector<pair<int, int>> goals;
    vector<pair<int, int>> helper_parkings;
    string filename = "maps/" + string(argv[1]) + ".txt";

    // Load map
    if(!utils::loadMap(filename, map, starts, goals, helper_parkings)){
        cout << "Error loading map" << endl;
        return 1;
    }

    // Print map
    utils::printMap(map, starts, goals, helper_parkings);

    // Call prioritized planning
    PrioritizedPlanning p(map, starts, goals, helper_parkings);
    vector<Result> results = p.solve();

    // Save results
    if(!results.empty()){
        utils::saveSolution(results, string(argv[1]));
    }

    return 0;
}