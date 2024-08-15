#ifndef HCCBS_H
#define HCCBS_H
 
#include "MAPF.h"
#include "utils/HierarchicalSolve.h"

#include <algorithm> 
#include <chrono> 
#include <functional> 
#include <queue>
#include <random>
#include <stdexcept>

#include "CBS.h"
#include "detection/GridCD.h"
#include "utils/MotionConstraint.h"
#include "utils/CTNode.h"
#include "utils/CTSolve.h"

/*
This is an implementation of Hierarchical Composition Conflict-Based Search in a time-extended grid world, assuming a 4-neighbor model. 
Original paper can be found here: https://ieeexplore.ieee.org/iel7/7083369/7339444/09483630.pdf
*/
class HCCBS : public MAPF{
    
    public: 

        using FullConstraint    = std::pair<size_t, MotionConstraint>;                              // alias for a full constraint that holds the agent and the motion constraint to be applied 
        using Node              = CTNode<MotionConstraint>;                                         // alias for a constraint tree node that uses motion constraints
        using ConstraintMap     = std::map<size_t, std::set<MotionConstraint>>;                     // alias for a map that tracks agent's and their applied motion constraints
        using LowLevelFunction  = std::function<std::pair<bool, std::vector<Coord>>                              
                                                (const Coord&, const Coord&,  
                                                 const std::set<MotionConstraint>&, 
                                                 size_t)>;                                          // alias for a functor for the low-level search function
        using Subproblem        = std::vector<size_t>;                                              // alias for a subproblem consisting of a list of agent ids                                                                    
        using Layer             = std::vector<Subproblem>;                                          // alias for a layer consisting of a list of subproblems                                                                 
        using ConstraintTree    = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>; // alias for a constraint tree node that is represented using a priority queue
        using NodeReference     = std::map<size_t, std::vector<Node>*>;                             // alias for a map that tracks a constraint tree's id and all its leaf nodes

    protected: 

        LowLevelFunction                      m_lowlevel;               // low level search algorithm used to find individual agent paths
        std::map<size_t, MotionTask>          m_tasks;                  // maps agent ID to assigned motion task
        std::vector<size_t>                   m_agents;                 // list of all agent IDs
        std::string                           m_heuristic;              // "random", "conflict", "cardinal" options for agent group composition functions
        std::map<Subproblem, Node>            m_solutions;              // holds the solution node of each CT search
        size_t                                m_exploredCount;          // tracks the number of explored nodes across all CTs
        size_t                                m_totalCount;             // tracks the number of total nodes across all CTs
        std::map<Subproblem, ConstraintTree*> m_treeReference;          // saves the returned CTs from each search iteration
        std::map<Subproblem, NodeReference*>  m_nodeReference;          // saves the nodes from each subproblem grouping after cross trees
        CBS                                   m_cbs;                    // CBS object used to solve each subproblem 
        double                                m_crossTime = 0;          // tracks the amount of time spent cross CTs - for improving implementation purposes

    private: 

        size_t                                m_nodeReferenceIndex = 0; // counter that determines the index of the constraint tree for the node reference condensed representation
        std::string                           m_name = "HCCBS"; 

    public: 
        
        HCCBS() : 
            MAPF(), 
            m_heuristic("random") {}; 

        HCCBS(LowLevelFunction& _lowlevel, std::string _cost = "soc", std::string _heuristic = "random", bool _debug = false) : 
            m_lowlevel(_lowlevel), 
            MAPF(_cost, _debug), 
            m_heuristic(_heuristic){}; 

        ~HCCBS() override{
            for (auto kv : m_treeReference) {
                if (kv.second != nullptr) 
                    delete kv.second; 
            }
        }
        
        // Resets the search parameters to their default state.
        void Reset() override; 
        
        // Resets the search parameters and initializes the tasks and agents.
        void Reset(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents); 

        // Traditional HCCBS solve function that handles multi-agent pathfinding.
        MultiPathSolution* Solve(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents) override;

    protected: 

        // Organizes a layer of subproblems based on the selected heuristic.
        Layer OrganizeLayer(const Layer& _layer); 

        // Retrieves subproblems from a given layer, returning only two at a time.
        Layer GetSubproblems(const Layer& _layer, size_t _index); 

        // Calculates the minimum end time for the low-level search based on the last applied motion constraint.
        size_t FindMinimumEndTime(size_t _agent, const std::set<MotionConstraint>& _constraints); 

        // Counts motion conflicts between subproblems, useful for conflict and cardinal conflict heuristics.
        std::map<std::pair<Subproblem, Subproblem>, size_t> CountMotionConflicts(const Layer& _layer, bool _cardinal); 

        // Organizes agents within a layer based on conflict counts, used for conflict-based heuristics.
        void OrganizeAgents(const Layer& _layer, Layer& _organized, std::map<std::pair<Subproblem, Subproblem>, size_t>& _conflictCounter); 

    private: 

        // Initializes the first layer of subproblems based on the full problem, typically one agent per subproblem.
        Layer InitializeLayer(Subproblem _fullProblem); 

        // Crosses the trees from two given subproblems to create a new subproblem.
        Subproblem CrossTrees(const Subproblem& _groupOne, const Subproblem& _groupTwo); 

};

#endif