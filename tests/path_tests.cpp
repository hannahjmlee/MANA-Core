#include "Problem.h"
#include "planning/path/AStar.h"
#include "planning/path/Dijkstra.h"
#include "utils/MotionConstraint.h"


#include <iostream>

void TestOptimalPathfinders(std::string _filename, size_t _scenario, bool _debug = false) {
    // compares Dijkstra and AStar Solutions

    Problem problemInstance = Problem(_filename, _scenario); 
    auto grid = problemInstance.GetGrid(); 
    std::cout << "Testing Optimal Pathfinders..." << std::endl; 

    // get eleventh motion task from a map and scenario
    auto task = problemInstance.GetTasks()[10]; 
    auto start = task.first; 
    auto goal = task.second; 
    std::cout << "\tMotion Task:"; 
    std::cout << "(" << start.first << ", " << start.second << ") --> "; 
    std::cout << "(" << goal.first << ", " << goal.second << ")" << std::endl; 

    // Create A* and Dijkstra objects
    AStar astar(grid, _debug); 
    Dijkstra dijkstra(grid, _debug); 

    // No constraints are applied to the search
    std::set<MotionConstraint> constraints; 
    
    // Solve the motion query
    auto [successD, dijkstraPath] = dijkstra.Solve(start, goal, constraints, 0); 
    auto [successA, astarPath] = astar.Solve(start, goal, constraints, 0); 

    // print out paths
    if (_debug) {
        std::cout << "\tA* Path:" << std::endl << "\t\t"; 
        for (auto vertex : astarPath) {
            std::cout << "(" << vertex.first <<", " << vertex.second << ") "; 
        }
        std::cout << std::endl; 

        std::cout << "\tDijkstra Path:" << std::endl << "\t\t"; 
        for (auto vertex : dijkstraPath) {
            std::cout << "(" << vertex.first <<", " << vertex.second << ") "; 
        }
    }

    // compare path lengths
    if (astarPath.size() != dijkstraPath.size()) {
        std::cout << "\tERROR: paths are different lengths" << std::endl; 
    } else {
        std::cout << "\tDijkstra and A* returned the optimal solution: " << astarPath.size() - 1 << std::endl; 
    }
    std::cout << std::endl; 
}