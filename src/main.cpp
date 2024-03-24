#include "prioritized.hpp"

int main(int argc, char* argv[]){

    // Initialize variables
    vector<vector<int>> map;
    vector<pair<int, int>> starts;
    vector<pair<int, int>> goals;
    pair<int, int> helper_parking;
    string filename = "maps/" + string(argv[1]) + ".txt";

    // Load map
    if(!utils::loadMap(filename, map, starts, goals, helper_parking)){
        cout << "Error loading map" << endl;
        return 1;
    }

    // Print map
    utils::printMap(map, starts, goals, helper_parking);

    // Call prioritized planning
    PrioritizedPlanning p(map, starts, goals, helper_parking);
    p.solve();

    return 0;
}