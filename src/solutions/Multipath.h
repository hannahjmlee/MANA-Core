#ifndef MULTIPATH_SOLUTION_H
#define MULTIPATH_SOLUTION_H

#include "utils/CTNode.h"

#include <map> 
#include <string> 
#include <utility>
#include <vector> 


class MultiPathSolution{
    public: 

        using Coord   = std::pair<size_t, size_t>;              // alias for a coordinate pair (x, y) 
        using PathMap = std::map<size_t, std::vector<Coord>>;   // alias for a map that tracks agent index to its solution path

    private: 

        PathMap     m_solutions;                    // solution to the MAPF problem
        size_t      m_solutionCost = 0;             // solution cost of MAPF problem
        bool        m_success = false;              // true if solution found, false otherwise
        size_t      m_searchSize = 0;               // search space size
        size_t      m_spaceSize = 0;                // state space size
        std::string m_name = "MultiPathSolution";   // name of MultiPathSolution instance

    public: 

        // Default constructor
        MultiPathSolution(){}; 

        // Destructor
        ~MultiPathSolution() = default; 

        // Copy constructor using pointer
        MultiPathSolution(const MultiPathSolution* _other) : 
            m_solutions(_other->m_solutions), 
            m_solutionCost(_other->m_solutionCost), 
            m_success(_other->m_success), 
            m_searchSize(_other->m_searchSize), 
            m_spaceSize(_other->m_spaceSize) {};

        // Copy constructor
        MultiPathSolution(const MultiPathSolution& _other) : 
            m_solutions(_other.m_solutions), 
            m_solutionCost(_other.m_solutionCost), 
            m_success(_other.m_success), 
            m_searchSize(_other.m_searchSize), 
            m_spaceSize(_other.m_spaceSize) {};
        
        // Constructor with initialization parameters
        MultiPathSolution(const PathMap& _solution, size_t _solutionCost, 
                          size_t _exploredCount = 0, size_t _totalCount = 0)  : 
            m_solutions(_solution), 
            m_solutionCost(_solutionCost), 
            m_success(true), 
            m_searchSize(_exploredCount), 
            m_spaceSize(_totalCount) {};

        // Assignment operator
        MultiPathSolution& operator=(const MultiPathSolution& _other) {
            if (this == &_other) {
                return *this;
            }

            m_solutions = _other.m_solutions;
            m_solutionCost = _other.m_solutionCost;
            m_success = _other.m_success; 

            m_searchSize = _other.m_searchSize; 
            m_spaceSize = _other.m_spaceSize; 

            return *this;
        };

        // Method to get the solution paths
        std::map<size_t, std::vector<Coord>> GetSolution() const{
            return m_solutions; 
        };

        // Method to get the solution cost
        size_t GetSolutionCost() const{
            return m_solutionCost; 
        };

        // Method to check if the solution was successful
        bool GetSuccess() const {
            return m_success; 
        };

        // Method to get the search size (number of nodes explored)
        size_t GetSearchSize() const {
            return m_searchSize; 
        }; 

        // Method to get the space size (total number of nodes)
        size_t GetSpaceSize() const {
            return m_spaceSize; 
        };

        // Method to get the name of the solution instance
        std::string GetName() const {
            return m_name; 
        };

}; 

#endif 
