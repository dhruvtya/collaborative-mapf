#include "prioritized.hpp"
#include "cbs.hpp"

int main(int argc, char* argv[]){

    // Check if the correct arguments are passed
    if(argc != 3){
        cout << "Arguments required: <algorithm> <map_name>" << endl;
        return 1;
    }
    if(string(argv[1]) != "cbs" && string(argv[1]) != "prioritized"){
        cout << "Invalid algorithm" << endl;
        return 1;
    }

    // Initialize variables
    vector<vector<int>> map;
    vector<pair<int, int>> starts;
    vector<pair<int, int>> goals;
    vector<pair<int, int>> helper_parkings;
    string filename = "maps/" + string(argv[2]) + ".txt";

    // Load map
    if(!utils::loadMap(filename, map, starts, goals, helper_parkings)){
        cout << "Error loading map" << endl;
        return 1;
    }

    // Print map
    utils::printMap(map, starts, goals, helper_parkings);

    vector<Result> results;
    if(string(argv[1]) == "cbs"){
        // Call CBS
        CBS c(map, starts, goals, helper_parkings);
        results = c.solve();
    }
    else if(string(argv[1]) == "prioritized"){
        // Call Prioritized Planning
        PrioritizedPlanning p(map, starts, goals, helper_parkings);
        results = p.solve();
    }

    // Save results
    if(!results.empty()){
        utils::saveSolution(results, string(argv[1]) + "_" + string(argv[2]));
    }
    else{
        return 1;
    }

    return 0;
}