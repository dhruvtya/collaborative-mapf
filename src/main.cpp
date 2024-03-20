#include "prioritized.hpp"

int main(){

    // Test prioritized planning
    vector<vector<int>> map = {{0, 0, 0, 0, 0},
                               {0, 1, 1, 1, 0},
                               {0, 1, 0, 1, 0},
                               {0, 1, 1, 1, 0},
                               {0, 0, 0, 0, 0}};

    vector<pair<int, int>> starts = {{1, 1}, {1, 3}};
    vector<pair<int, int>> goals = {{3, 1}, {3, 3}};
    pair<int, int> helper_parking = {1, 4};
    
    PrioritizedPlanning p(map, starts, goals, helper_parking);
    p.solve();
    
    return 0;
}