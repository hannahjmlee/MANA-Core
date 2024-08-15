#ifndef PBS_H
#define PBS_H
 
#include "MAPF.h"

#include <algorithm> 
#include <chrono> 
#include <functional> 
#include <stdexcept>

#include "detection/GridCD.h"
#include "utils/CTNode.h"
#include "utils/CTSolve.h"
#include "utils/MotionConstraint.h"

/*
This is an implemenation of Priority-Based search in a time-extended grid world, assuming a 4-neighbor movement model. 
Original paper can be found here: https://aaai.org/ojs/index.php/AAAI/article/download/4758/4636
*/
class PBS : public MAPF {

    public: 

        using Coord                = std::pair<size_t, size_t>;                                       // alias for a coordinate pair (x, y)
        using MotionTask           = std::pair<Coord, Coord>;                                         // alias for a motion task consisting of a start and goal coordinate
        using PriorityConstraint   = size_t;                                                          // alias for a priority constraint represented by a size_t
        using FullConstraint       = std::pair<size_t, PriorityConstraint>;                           // alias for a full constraint that includes the agent ID and priority constraint
        using Node                 = CTNode<PriorityConstraint>;                                      // alias for a constraint tree node that uses priority constraints
        using PathMap              = std::map<size_t, std::vector<Coord>>;                            // alias for a map that tracks agent index to its solution path
        using ConstraintMap        = std::map<size_t, std::set<PriorityConstraint>>;                  // alias for a map that tracks agents and their applied priority constraints
        using LowLevelFunction     = std::function<std::pair<bool, std::vector<Coord>>                             
                                                   (const Coord&, const Coord&, 
                                                    const std::set<MotionConstraint>&, 
                                                    size_t)>;                                         // alias for a functor for the low-level search function

    private: 
 
        std::string                   m_name = "PBS";                // name of the PBS instance
        LowLevelFunction              m_lowlevel;                    // low-level search algorithm used to find individual agent paths
        std::map<size_t, MotionTask>  m_tasks;                       // maps agent ID to assigned motion task
        std::vector<size_t>           m_agents;                      // list of all agent IDs
        size_t                        m_exploredCount;               // used to track the number of explored CT nodes
        size_t                        m_totalCount;                  // used to track the size of the CT when the search is terminated
        std::vector<Node>             m_root = std::vector<Node>();  // PBS starts with an empty root 

    public: 

        PBS() : MAPF() {}; 

        PBS(LowLevelFunction& _lowlevel) : 
            MAPF(), 
            m_lowlevel(_lowlevel) {}; 

        PBS(LowLevelFunction& _lowlevel, std::string _cost, bool _debug = false) : 
            m_lowlevel(_lowlevel), 
            MAPF(_cost, _debug) {}; 

        ~PBS() override = default; 
        
        // Resets the search parameters to their default state.
        void Reset() override; 

        // Resets the search parameters and initializes the tasks and agents.
        void Reset(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents); 

        // Traditional PBS solve function that handles multi-agent pathfinding.
        MultiPathSolution* Solve(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents) override;

        // Sets the root from which PBS can begin its search.
        void SetRoot(const std::vector<Node>& _root); 

    private: 

        // Modified low-level search - expands priority constraints into motion constraints before calling low-level search.
        std::pair<bool, std::vector<Coord>> LowLevel(size_t _agent, const Node& _node); 

        // Returns the initial set of nodes from which PBS begins its search.
        std::vector<Node> Initialize(); 

        // Checks a given CT node for constraints and returns any that apply.
        std::vector<FullConstraint> Validate(const Node& _node); 

        // Creates children nodes based on a parent node and the provided constraints.
        std::vector<Node> Split(const Node& _parent, const std::vector<FullConstraint>& _constraints); 

        // Calculates the minimum end time for the low-level search based on the applied constraint.
        size_t FindMinimumEndTime(const Coord& _goal, const std::set<MotionConstraint>& _constraints); 

        // Checks the validity of a priority constraint by detecting constraint cycles.
        std::pair<bool, std::vector<size_t>> CheckConstraintValidity(size_t _agent, const ConstraintMap& _constraintMap); 

        // Debugging function - checks that the returned path from the low-level search adheres to the constraints.
        bool PathValidate(const Node& _pathA, size_t _agent); 

};

#endif