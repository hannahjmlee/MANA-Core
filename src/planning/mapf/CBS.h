#ifndef CBS_H
#define CBS_H
 
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
This is an implementation of Conflict-Based Search in a time-extended grid world, assuming a 4-neighbor model. 
Original paper can be found here: https://www.sciencedirect.com/science/article/pii/S0004370214001386
*/
class CBS : public MAPF {

    public: 

        using FullConstraint    = std::pair<size_t, MotionConstraint>;                              // alias for a full constraint that includes the agent ID and the motion constraint to be applied 
        using Node              = CTNode<MotionConstraint>;                                         // alias for a constraint tree node that uses motion constraints
        using ConstraintMap     = std::map<size_t, std::set<MotionConstraint>>;                     // alias for a map that tracks agents and their applied motion constraints
        using LowLevelFunction  = std::function<std::pair<bool, std::vector<Coord>>                             
                                                (const Coord&, const Coord&, 
                                                 const std::set<MotionConstraint>&, 
                                                 size_t)>;                                          // alias for a functor for the low-level search function

        using ConstraintTree    = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>; // alias for a constraint tree represented using a priority queue
        using NodeReference     = std::map<size_t, std::vector<Node>*>;                             // alias for a map that tracks a constraint tree's ID and all its leaf nodes

    private:  
 
        std::string                  m_name = "CBS";                // Name of the CBS instance
        LowLevelFunction             m_lowlevel;                    // Low-level search algorithm used to find individual agent paths
        std::map<size_t, MotionTask> m_tasks;                       // Map of agent ID to assigned motion tasks
        std::vector<size_t>          m_agents;                      // List of all agent IDs
        size_t                       m_exploredCount = 0;           // Tracks the number of explored CT nodes
        size_t                       m_totalCount = 0;              // Tracks the size of the CT when the search is terminated
        std::vector<Node>            m_root = std::vector<Node>();  // CBS starts with an empty root
        NodeReference*               m_nodeReference = nullptr;     // Used for other MAPF algorithms that may condense the node representation

    public: 

        CBS() : MAPF() {}; 

        CBS(LowLevelFunction& _lowlevel, std::string _cost = "soc", bool _debug = false) : 
            m_lowlevel(_lowlevel), 
            MAPF(_cost, _debug) {}; 

        // Resets the search parameters. 
        void Reset() override; 

        // Resets the search parameters and sets the current set of agents.
        void Reset(const std::vector<size_t>& _agents); 

        // Resets the search parameters and sets the current set of agents and tasks.
        void Reset(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents); 

        // Traditional CBS
        MultiPathSolution* Solve(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents) override; 

        // Begins CBS from the given constraint tree
        Node Solve(const std::vector<size_t>& _agents, ConstraintTree* _tree);

        // Begins CBS from the given constraint tree with a condensed node 
        // This does not track the number of total nodes - if you need it, you'll need to add it!
        Node Solve(const std::vector<size_t>& _agents, const NodeReference* _nodeReference, ConstraintTree* _tree, size_t& _exploredCount) const;

        // Returns the number of explored nodes from the most recent CBS call
        size_t GetExploredCount() const; 

        // Sets the node reference for expanding condensed node representations
        void SetNodeReference(NodeReference* _nodeReference); 

        // Sets the root from which CBS can begin its search
        void SetRoot(const std::vector<Node>& _root); 

        // Sets the motion tasks that CBS uses
        void SetTasks(const std::map<size_t, MotionTask> _tasks);

    private: 

        // Returns the initial set of nodes from which CBS begins its search
        std::vector<Node> Initialize(const std::vector<size_t>& _agents); 

        // Checks a given CT node for constraints. Agents are passed in to allow users to choose which agents are validated
        std::vector<FullConstraint> Validate(const Node& _node, const std::vector<size_t>& _agents) const; 

        // Creates children based off a parent node and the provided constraints
        std::vector<Node> Split(const Node& _parent, 
                                  const std::vector<FullConstraint>& _constraints) const; 

        // Helper function that calculates the minimum end time for the low-level search based on the last applied motion constraint
        size_t FindMinimumEndTime(size_t _agent, const std::set<MotionConstraint>& _constraints) const; 
};

#endif // CBS_H