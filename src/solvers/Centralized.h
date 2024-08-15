#ifndef CENTRALIZED_MAPF_H
#define CENTRALIZED_MAPF_H

#include <chrono>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "Problem.h"
#include "planning/path/Pathfinder.h"
#include "planning/path/AStar.h"
#include "planning/path/Dijkstra.h"

#include "planning/mapf/MAPF.h"
#include "planning/mapf/CBS.h"
#include "planning/mapf/HCCBS.h"
#include "planning/mapf/PBS.h"

#include "solutions/Multipath.h"
#include "utils/MotionConstraint.h"


class CentralizedSolver{

    public:

        using Coord            = std::pair<size_t, size_t>;                       // alias for a coordinate pair (x, y)
        using MotionTask       = std::pair<Coord, Coord>;                         // alias for a motion task consisting of a start and goal coordinate
        using SolutionMap      = std::map<size_t, std::vector<Coord>>;            // alias for a map that tracks agent index to its solution path
        using LowLevelFunction = std::function<std::pair<bool, std::vector<Coord>>
                                            (const Coord&, const Coord&,
                                            const std::set<MotionConstraint>&,
                                            size_t)>;                              // alias for a functor for the low-level search function

    public:

        double m_runtime;                                                         // runtime of the centralized solver

    private:

        bool m_debug;                                                             // debug flag to enable/disable debug outputs
        std::string m_costMetric;                                                 // cost metric options (e.g., "soc", "makespan")
        std::string m_heuristic;                                                  // heuristic type (e.g., "random", "conflict", "cardinal")

        Pathfinder* m_lowlevel = nullptr;                                         // pointer to the low-level pathfinding algorithm
        MAPF* m_mapf = nullptr;                                                   // pointer to the MAPF algorithm
        Problem* m_problem = nullptr;                                             // pointer to the problem instance
        MultiPathSolution* m_solution = nullptr;                                  // pointer to the solution object

        std::map<size_t, MotionTask> m_tasks;                                     // map of agent ID to assigned motion tasks
        std::vector<size_t> m_agents;                                             // list of all agent IDs
        std::string m_name = "CentralizedSolver";                                 // name of the CentralizedSolver instance

    public:

        // Default constructor
        CentralizedSolver(){};

        // Constructor with initialization parameters
        CentralizedSolver(Problem* _problem, std::string _mapf = "cbs",
                          std::string _lowlevel = "AStar", std::string _costMetric = "soc",
                          bool _debug = false);

        // Constructor with initialization parameters
        CentralizedSolver(Problem* _problem, std::string _mapf = "cbs",
                          std::string _lowlevel = "AStar", std::string _costMetric = "soc",
                          std::string _heuristic = "random", bool _debug = false);

        // Destructor
        ~CentralizedSolver(){
            if (m_lowlevel != nullptr)
                delete m_lowlevel;
            if (m_mapf != nullptr)
                delete m_mapf;
            if (m_solution != nullptr)
                delete m_solution;
        }

        // Creates pathfinding object
        void CreatePathfinder(std::string _lowlevel);

        // Creates MAPF objects
        void CreateMAPF(std::string _mapf);

        // Solve method for the centralized MAPF problem
        std::tuple<bool, size_t, SolutionMap, double> Solve(size_t _problemSize);

        // Method to get the analysis of work done during the solve process
        std::pair<size_t, size_t> GetWorkAnalysis() const;

        // Method to get the name of the CentralizedSolver instance
        std::string GetName() const {
            return m_name;
        };

};

#endif
