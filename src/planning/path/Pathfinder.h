#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <array>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "utils/MotionConstraint.h"
#include "utils/SpatialMotionConstraint.h"
#include "Problem.h"

class Pathfinder {

    public:

        using Coord = std::pair<double, double>;    // alias for a coordinate pair (x, y)

    protected:

        bool                                 m_debug = false;        // debug flag to enable/disable debug outputs
        double                               m_resolution = 1; 

    private:

        std::map<Coord, std::vector<Coord>*> m_neighbors;            // map storing the neighbors of each coordinate in the grid
        std::string                          m_name = "Pathfinder";  // name of the Pathfinder instance

    public:

        // destructor for Pathfinder (defaulted)
        virtual ~Pathfinder() = default;

        // default constructor
        Pathfinder() {};

        // constructor with debug flag
        Pathfinder(bool _debug) :
            m_debug(_debug) {};

        // constructor with grid initialization and optional debug flag
        Pathfinder(Problem* _problem, bool _debug = false) {
            m_debug = _debug;
            m_resolution = _problem->m_resolution;
            InitializeNeighbors(_problem);
        };

        // constructor with predefined neighbors map and optional debug flag
        Pathfinder(const std::map<Coord, std::vector<Coord>*> _neighbors, bool _debug = false) :
            m_neighbors(_neighbors),
            m_resolution(1),
            m_debug(_debug) {};

        // copy constructor
        Pathfinder(const Pathfinder& _other) :
            m_debug(_other.m_debug),
            m_neighbors(_other.m_neighbors) {};

        // copy constructor using a pointer to another Pathfinder instance
        Pathfinder(const Pathfinder* _other) :
            m_debug(_other->m_debug),
            m_neighbors(_other->m_neighbors) {};

        // Method to initialize the neighbors for each grid cell
        void InitializeNeighbors(Problem* _problem);

        // Method to retrieve the entire neighbors map
        std::map<Coord, std::vector<Coord>*> GetNeighbors();

        // Method to retrieve the neighbors of a specific vertex (coordinate)
        std::vector<Coord> GetNeighbors(const Coord& _vertex) const;

        // Virtual solve method to be overridden by derived classes
        virtual std::pair<bool, std::vector<Coord>> Solve(const Coord& _start, const Coord& _goal,
                                                        const std::vector<MotionConstraint>& _constraints = std::vector<MotionConstraint>(),
                                                        size_t _endtime = 0) const {
            std::vector<Coord> empty;
            return {false, empty};
        };

        virtual std::pair<bool, std::vector<Coord>> Solve(const Coord& _start, const Coord& _goal,
                                                        const std::vector<SpatialMotionConstraint>& _constraints = std::vector<SpatialMotionConstraint>(),
                                                        size_t _endtime = 0) const {
            std::vector<Coord> empty;
            return {false, empty};
        };

        // Method to get the name of the Pathfinder instance
        std::string GetName() const {
            return m_name;
        };


};

// Inline method implementations

// InitializeNeighbors: Set up the neighbors for each cell in the grid
inline
void
Pathfinder::
InitializeNeighbors(Problem* _problem) {
    std::set<Coord> validCoords = _problem->GetVertices();
    double step = _problem->GetStepSize();

    auto bounds = _problem->GetUpperBounds();
    double max_x = bounds.first;
    double max_y = bounds.second;

    m_neighbors.clear();

    for (const auto& coord : validCoords) {
      std::vector<Coord>* validNeighbors = new std::vector<Coord>();
      auto x = coord.first;
      auto y = coord.second;
      std::array<Coord, 4> neighbors = {std::make_pair(x - step, y),  // Left
                                        std::make_pair(x + step, y),  // Right
                                        std::make_pair(x, y - step),  // Up
                                        std::make_pair(x, y + step)}; // Down

      for (const auto& neighbor : neighbors) {
          if (neighbor.first < 0 || neighbor.first > max_x)
              continue;
          else if (neighbor.second < 0 || neighbor.second > max_y)
              continue;
          else if (validCoords.count(neighbor) == 0)
              continue;

          validNeighbors->push_back(neighbor);
      }

      validNeighbors->push_back({x, y});
      m_neighbors[{x, y}] = validNeighbors;
    }

    if (m_debug) {
        std::cout << "Neighbor Check for Graph (first 20 vertices only):" << std::endl;
        size_t count = 0; 
        for (auto kv : m_neighbors) {
            if (count > 20) break; 
            count++; 
            std::cout << "\t(" << kv.first.first << ", " << kv.first.second << "): ";
            for (auto z : *(kv.second)) {
                std::cout << "(" << z.first << ", " << z.second << ")  ";
            }
            std::cout << std::endl;
        }
    }


    return;
}

// GetNeighbors: Return the neighbors of a specific vertex
inline
std::vector<Pathfinder::Coord>
Pathfinder::
GetNeighbors(const Coord& _vertex) const {
    return *(m_neighbors.at(_vertex));
}

// GetNeighbors: Return the entire neighbors map
inline
std::map<Pathfinder::Coord, std::vector<Pathfinder::Coord>*>
Pathfinder::
GetNeighbors() {
    return m_neighbors;
}

#endif // PATHFINDER_H
