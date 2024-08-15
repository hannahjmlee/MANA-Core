#include <iostream> 
#include <vector>
#include <string> 

#include "utils_tests.cpp"
#include "path_tests.cpp"
#include "mapf_tests.cpp"


int main(int argc, char *argv[]) {
    /*
    This is an interface for specifically for the "An Analysis of Constraint-Based Multi-Agent Pathfinding Algorithms" 
    paper. When running the cbs_analysis executable:
        ./cbs_analysis.exe [map name] [scenario number] [number of agents] [algorithm]
    it will return the following output: 
        [map name],[scenario number],[algorithm],[number of agents],[solve time],[solution cost],[explored nodes],[total nodes]
    */

    if (argc != 5) {
        std::cout << "Incorrect number of arguments. Please follow this format when running this executable." << std::endl;
        std::cout << "\t./cbs_analysis.exe [map name] [scenario number] [number of agents] [algorithm]" << std::endl;
        std::cout << "\t\t[algorithm] = (CBS, PBS)" << std::endl;
        std::cout << std::endl; 
        std::cout << "The output will be a string in the form of: " << std::endl;
        std::cout << "\t[map name],[scenario number],[algorithm],[number of agents],[solve time],[solution cost],[explored nodes],[total nodes]" << std::endl;

        return 0; 
    }
    std::string mapName = argv[1]; 
    size_t scenarioNum = std::stoull(argv[2]); 
    size_t numAgents = std::stoull(argv[3]); 
    std::string mapf = argv[4]; 


    // create problem instance
    Problem* problemInstance = new Problem(mapName, scenarioNum); 
    
    // create solver
    CentralizedSolver solver (problemInstance, mapf, "AStar", "soc", false); 

    // solve problem instance
    auto [success, solutionCost, solution, solveTime] = solver.Solve(numAgents); 

    // extract tracked data values
    auto nodeCounts = solver.GetWorkAnalysis(); 
    auto exploredNodes = nodeCounts.first; 
    auto totalNodes = nodeCounts.second; 

    // output solution ad algorithm data
    std::cout << mapName << ",";
    std::cout << scenarioNum << ","; 
    std::cout << mapf << ","; 
    std::cout << numAgents << ","; 
    std::cout << solveTime << ","; 
    std::cout << solutionCost << ","; 
    std::cout << exploredNodes << ","; 
    std::cout << totalNodes << std::endl; 

    return 0; 
}