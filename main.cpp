#include <iostream>
#include <vector>
#include <string>

#include "tests/utils_tests.cpp"
#include "tests/path_tests.cpp"
#include "tests/mapf_tests.cpp"


int main() {
    std::vector<std::string> maps = {"empty-8-8", "empty-16-16", "empty-32-32", "empty-48-48",
                 "random-32-32-10", "random-32-32-20", "random-64-64-10", "random-64-64-20",
                 "room-32-32-4", "room-64-64-8", "room-64-64-16",
                 "maze-32-32-2", "maze-32-32-4", "maze-128-128-2", "maze-128-128-10",
                 "Berlin_1_256", "Boston_0_256", "Paris_1_256",
                 "ht_chantry", "ht_mansion_n", "lak303d", "lt_gallowstemplar_n",
                 "den312d", "ost003d", "brc202d", "den520d", "w_woundedcoast"};

    std::string mapName = "empty-8-8";
    size_t scenarioNum = 1;
    size_t numAgents = 5; 
    
    std::string mapf = "CBS"; // current support: CBS, PBS, HCCBS
    std::string mapfHeuristic = "cardinal"; // for HCCBS

    bool debug = false;
    // bool debug = true;

    TestMultiAgentPathfinders(mapName, scenarioNum, numAgents, mapf, "AStar", mapfHeuristic, debug);

    return 0;
}
