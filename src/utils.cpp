#include "utils.hpp"

bool utils::loadMap(string &filename, vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, pair<int, int> &helper_parking){

    // Read file
    ifstream MyFile(filename);
    string line;
    int rows = 0;
    int cols = 0;

    // First line is the number of rows and columns
    getline(MyFile, line);
    rows = stoi(line.substr(0, line.find(" ")));
    cols = stoi(line.substr(line.find(" ") + 1, line.size()));

    // Read map
    map.clear();
    for(int i = 0; i < rows; i++){
        getline(MyFile, line);
        vector<int> row;
        for(int j = 0; j < line.size(); j++){
            if(line[j] == '.'){
                row.push_back(0);
            }
            else if(line[j] == '@'){
                row.push_back(-1);
            }
            else if(line[j] == '~'){
                row.push_back(1);
            }
        }
        map.push_back(row);
    }

    // Read number of transit agents
    getline(MyFile, line);
    int num_transit_agents = stoi(line);

    // Read transit agents
    // Format: start_row start_col goal_row goal_col
    starts.clear();
    goals.clear();
    for(int i = 0; i < num_transit_agents; i++){
        getline(MyFile, line);
        int start_row = stoi(line.substr(0, line.find(" ")));
        int start_col = stoi(line.substr(line.find(" ") + 1, line.find(" ", line.find(" ") + 1)));
        int goal_row = stoi(line.substr(line.find(" ", line.find(" ") + 1) + 1, line.rfind(" ")));
        int goal_col = stoi(line.substr(line.rfind(" ") + 1, line.size()));
        starts.push_back(make_pair(start_row, start_col));
        goals.push_back(make_pair(goal_row, goal_col));
    }

    // Read helper parking
    getline(MyFile, line);
    int helper_parking_row = stoi(line.substr(0, line.find(" ")));
    int helper_parking_col = stoi(line.substr(line.find(" ") + 1, line.size()));
    helper_parking = make_pair(helper_parking_row, helper_parking_col);

    // Close file
    MyFile.close();


    // Sanity check
    for(int i = 0; i < map.size(); i++){
        if(map[i].size() != cols){
            cout << "Error: Map is not rectangular" << endl;
            return false;
        }
    }

    if(starts.size() != num_transit_agents || goals.size() != num_transit_agents){
        cout << "Error: Number of starts and goals do not match" << endl;
        return false;
    }

    cout << "Map loaded successfully" << endl;

    return true;
}

void utils::printMap(const vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, pair<int, int> &helper_parking){
    cout << "-----------------------------" << endl;
    
    for(int i = 0; i < map.size(); i++){
        for(int j = 0; j < map[i].size(); j++){
            if(map[i][j] == 0){
                cout << ".";
            }
            else if(map[i][j] == -1){
                cout << "@";
            }
            else if(map[i][j] == 1){
                cout << "~";
            }
            cout << " ";
        }
        cout << endl;
    }

    cout << "\nNum. of transit agents: " << starts.size() << endl;

    cout << "Starts: ";
    for(int i = 0; i < starts.size(); i++){
        cout << "(" << starts[i].first << ", " << starts[i].second << ") ";
    }
    cout << endl;

    cout << "Goals: ";
    for(int i = 0; i < goals.size(); i++){
        cout << "(" << goals[i].first << ", " << goals[i].second << ") ";
    }
    cout << endl;

    cout << "Helper parking: (" << helper_parking.first << ", " << helper_parking.second << ")" << endl;
    cout << "-----------------------------" << endl;
}

void utils::printHeuristicMap(const vector<vector<int>> &map){
    cout << "-----------------------------" << endl;
    for(int i = 0; i < map.size(); i++){
        for(int j = 0; j < map[i].size(); j++){
            cout << map[i][j] << " ";
        }
        cout << endl;
    }
    cout << "-----------------------------" << endl;
}

void utils::saveSolution(){

}