#ifndef MULTIPATH_SOLUTION_H
#define MULTIPATH_SOLUTION_H

#include "utils/CTNode.h"

#include <map>
#include <string>
#include <utility>
#include <vector>


class MultiPathSolution{
    public:

        using Coord   = std::pair<double, double>;              // alias for a coordinate pair (x, y)
        using PathMap = std::map<size_t, std::vector<Coord>>;   // alias for a map that tracks agent index to its solution path

    private:

        PathMap     m_solutions;                    // solution to the MAPF problem
        double      m_solutionCost = 0;             // solution cost of MAPF problem
        bool        m_success = false;              // true if solution found, false otherwise
        size_t      m_searchSize = 0;               // search space size
        size_t      m_spaceSize = 0;                // state space size
        size_t      m_resolution = 1;                // state space size
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
            m_resolution(_other->m_resolution),
            m_success(_other->m_success),
            m_searchSize(_other->m_searchSize),
            m_spaceSize(_other->m_spaceSize) {};

        // Copy constructor
        MultiPathSolution(const MultiPathSolution& _other) :
            m_solutions(_other.m_solutions),
            m_solutionCost(_other.m_solutionCost),
            m_resolution(_other.m_resolution),
            m_success(_other.m_success),
            m_searchSize(_other.m_searchSize),
            m_spaceSize(_other.m_spaceSize) {};

        // Constructor with initialization parameters
        MultiPathSolution(const PathMap& _solution, double _solutionCost,
                          size_t _resolution = 1, size_t _exploredCount = 0,
                          size_t _totalCount = 0)  :
            m_solutions(_solution),
            m_solutionCost(_solutionCost),
            m_resolution(_resolution),
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
            m_resolution = _other.m_resolution;

            m_searchSize = _other.m_searchSize;
            m_spaceSize = _other.m_spaceSize;

            return *this;
        };

        // Method to get the solution paths
        std::map<size_t, std::vector<Coord>> GetSolution() const{
            return m_solutions;
        };

        // Method to get the solution cost
        double GetSolutionCost() const{
            return static_cast<double>(m_solutionCost / m_resolution);
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

        void PrintSolution() const {
            std::cout << "Solution:" << std::endl; 
            for (auto kv : m_solutions) {
                std::cout << "Agent " << kv.first << ": " << kv.second.size() << std::endl;
                std::cout << "\t"; 
                for (auto pos : kv.second) {
                    std::cout << "(" << pos.first <<", " << pos.second << ") "; 
                }
                std::cout << std::endl; 
            }
        }

};

#endif
