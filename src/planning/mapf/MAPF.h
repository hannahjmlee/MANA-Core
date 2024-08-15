#ifndef MAPF_H
#define MAPF_H

#include "solutions/Multipath.h"

#include <map> 
#include <string>
#include <set> 
#include <tuple> 
#include <utility> 
#include <vector> 

// Template class for multi-agent pathfinding algorithms.
class MAPF {

    public: 

        using Coord       = std::pair<size_t, size_t>;              // Alias for a coordinate pair (x, y)
        using MotionTask  = std::pair<Coord, Coord>;                // Alias for a motion task consisting of a start and goal coordinate
        using PathMap     = std::map<size_t, std::vector<Coord>>;   // Alias for a map that tracks agent index to its solution path

    protected: 

        std::string        m_cost;                  // Cost function option: "soc" (sum of costs) or "makespan"
        MultiPathSolution* m_solution = nullptr;    // Pointer to the multi-path solution object
        double             m_runtime;               // Runtime of the MAPF algorithm
        bool               m_debug = false;         // Debug flag for code development and debugging

    private: 

        std::string m_name = "MAPF";            

    public: 

        // Default constructor initializing cost to "soc"
        MAPF() : 
            m_cost("soc") {}; 

        // Constructor with specified cost function and optional debug flag
        MAPF(std::string _cost, bool _debug = false) :
            m_cost(_cost),
            m_debug(_debug) {};

        // Destructor, ensures solution object is deleted
        virtual ~MAPF(){
            if (m_solution != nullptr) {
                delete m_solution; 
            } 
        }

        // Virtual solve function to be implemented by derived classes
        virtual MultiPathSolution* Solve(const std::map<size_t, MotionTask>& _task, const std::vector<size_t>& _agents) {
            return m_solution; 
        };
        
        // Virtual reset function, can be overridden by derived classes
        virtual void Reset() {}; 

        // Get the runtime of the MAPF algorithm
        double GetRuntime() const; 
                
        // Get the name of the MAPF instance
        std::string GetName() const {
            return m_name; 
        };

    protected:

        // Calculate the cost of the paths based on the selected cost function
        size_t Cost(const PathMap& _paths) const; 

};

// Inline method implementations

// Cost: Calculate the total cost or makespan of the paths
inline
size_t
MAPF::
Cost(const PathMap& _paths) const {
    size_t cost = 0; 
    for (auto kv : _paths) {
        if (m_cost == "soc") {
            cost += kv.second.size() - 1; 
        } else {
            cost = std::max(cost, kv.second.size() - 1); 
        }
    }

    return cost; 
}

// GetRuntime: Return the runtime of the MAPF algorithm
inline
double 
MAPF:: 
GetRuntime() const {
    return m_runtime; 
}

#endif // MAPF_H
