#include "utils.hpp"

Agent::Agent(int id, AgentType type, pair<int, int> start, pair<int, int> goal, vector<vector<int>> heuristics)
                            : id_{id}, type_{type}, start_{start}, goal_{goal}, heuristics_{heuristics}, start_time_{0}{}

Agent::Agent(int id, AgentType type, pair<int, int> start, pair<int, int> goal, vector<vector<int>> heuristics, int start_time)
                            : id_{id}, type_{type}, start_{start}, goal_{goal}, heuristics_{heuristics}, start_time_{start_time}{}

bool Agent::operator<(const Agent& other) const{
    // Agents with lower ID have higher priority
    return this->id_ > other.id_;
}

bool Agent::operator==(const Agent& other) const{
    // Agents are equal if they have the same ID, Type, Start and Goal locations
    return (this->type_ == other.type_ && this->start_ == other.start_ && this->goal_ == other.goal_);
}

bool utils::loadMap(string &filename, vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, vector<pair<int, int>> &helper_parkings){

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
    int num_helper_agents = stoi(line);

    // Read helpers
    helper_parkings.clear();
    for(int i = 0; i < num_helper_agents; i++){
        getline(MyFile, line);
        int helper_parking_row = stoi(line.substr(0, line.find(" ")));
        int helper_parking_col = stoi(line.substr(line.find(" ") + 1, line.size()));
        helper_parkings.push_back(make_pair(helper_parking_row, helper_parking_col));
    }

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

void utils::printMap(const vector<vector<int>> &map, vector<pair<int, int>> &starts, vector<pair<int, int>> &goals, vector<pair<int, int>> &helper_parkings){
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

    cout << "Num. of helper agents: " << helper_parkings.size() << endl;
    for(int i = 0; i < helper_parkings.size(); i++){
        cout << "Helper "<< i + 1 << ": (" << helper_parkings[i].first << ", " << helper_parkings[i].second << ")" << endl;
    }
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

void utils::printResults(const vector<Result> &results){
    cout << "Printing results ===============" << endl;
    for(int i = 0; i < results.size(); i++){
        cout << "Agent ID: " << results[i].agent_id_ << endl;
        cout << "Type: " << ((int)results[i].type_?"Helper":"Transit") << endl;
        cout << "Start: " << results[i].start_.first << " " << results[i].start_.second << endl;
        cout << "Goal: " << results[i].goal_.first << " " << results[i].goal_.second << endl;
        cout << "Path: ";
        for(int j = 0; j < results[i].path_.size(); j++){
            cout << "(" << results[i].path_[j].first << ", " << results[i].path_[j].second << ") ";
        }
        if(i != results.size() - 1){
            cout << "\n ----------" << endl;
        }
        else{
            cout << endl;
        }
    }
    cout << "================================" << endl;
}
        
void utils::saveSolution(const vector<Result> &results, string filename){
    ofstream file;

    file.open("output/" + filename + ".csv");
    
    for(const auto &result : results){
        file << result.agent_id_ << "," << (int)result.type_ << "," << result.start_.first << "," << result.start_.second << "," << result.goal_.first << "," << result.goal_.second << ",";
        for(int i = 0; i < result.path_.size(); i++){
            file << result.path_[i].first << "," << result.path_[i].second;
            if(i != result.path_.size() - 1){
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
}

double utils::getSumOfCosts(const vector<vector<pair<int, int>>> &paths){
    double sum = 0;
    for(const auto &path : paths){
        sum += path.size();
    }
    return sum;
}

double utils::getSumOfCosts(const vector<Result> &results){
    double sum = 0;
    for(const auto &result : results){
        sum += result.path_.size();
    }
    return sum;
}

double utils::getSumOfCosts(const vector<vector<pair<int, int>>> &paths, const vector<vector<int>> &map){
    double sum = getSumOfCosts(paths);
    
    set<pair<int, int>> visited_mo;
    for(size_t i = 0; i < paths.size(); i++){
        for(size_t j = 0; j < paths[i].size(); j++){
            if(map[paths[i][j].first][paths[i][j].second] == 1 && visited_mo.find(paths[i][j]) == visited_mo.end()){
                sum += 10;
                visited_mo.insert(paths[i][j]);
            }
        }
    }

    return sum;
}

double utils::getManhattanDistance(const pair<int, int> &start, const pair<int, int> &goal){
    return abs(start.first - goal.first) + abs(start.second - goal.second);
}

int utils::getNumWaits(const vector<pair<int, int>>& path) {
    int waits = 0;
    for (int i = 1; i < path.size(); i++) {
        if (path[i-1] == path[i]) {
            waits++;
        }
    }
    return waits;
}


void utils::printPath(const vector<pair<int, int>>& path) {
    std::cout << "++++++++Printing Path++++++++++\n";
    for (auto it:path) {
        cout << it.first << ", " << it.second << endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++\n";

}