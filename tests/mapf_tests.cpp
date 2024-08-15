#include "Problem.h"
#include "solvers/Centralized.h"

#include <iostream> 

void TestMultiAgentPathfinders(std::string _filename, size_t _scenario, size_t _problemSize,
                               std::string _mapf, std::string _lowlevel, std::string _heuristic,
                               bool _debug = false) {
    std::cout << "Solving scen"<< _scenario << "_" << _filename << " with " 
              << _problemSize << " agents using " << _mapf << "..." << std::endl; 
                
    // create problem instance and solver
    Problem* problemInstance = new Problem(_filename, _scenario); 
    CentralizedSolver solver(problemInstance, _mapf, _lowlevel, "soc", _heuristic, _debug); 

    // solve problem instance
    auto [success, solutionCost, solution, solveTime] = solver.Solve(_problemSize); 

    // output solution
    if (!success) {
        std::cout << "\tThe solver could not find a solution - debugging necessary." << std::endl; 
    } else {
        std::cout << "\tSolution found." << std::endl; 
        if (_debug) {
            for (auto kv : solution) { 
                size_t agent = kv.first; 
                auto path = kv.second; 

                std::cout << "\t\tAgent " << agent << "'s Path: "; 
                for (auto cfg : path) {
                    std::cout << "(" << cfg.first << ", " << cfg.second << ")" << " "; 
                }
                std::cout << std::endl;
            }
        }
        std::cout << "\tSolution Cost: " << solutionCost << std::endl; 
        std::cout << "\tSolve Time: " << solveTime << std::endl; 

        auto nodeCounts = solver.GetWorkAnalysis(); 
        std::cout << "\tExplored Nodes: " << nodeCounts.first << std::endl; 
        std::cout << "\tTotal Nodes: " << nodeCounts.second << std::endl; 
    }
    std::cout << std::endl; 
    delete problemInstance; 
}