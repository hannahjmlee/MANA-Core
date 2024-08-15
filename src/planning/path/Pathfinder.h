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

class Pathfinder {

    public:

        using Coord = std::pair<size_t, size_t>;    // alias for a coordinate pair (x, y) 

    protected: 

        bool                                 m_debug = false;        // debug flag to enable/disable debug outputs

    private: 

        std::map<Coord, std::vector<Coord>*> m_neighbors;            // map storing the neighbors of each coordinate in the grid
        std::string                          m_name = "Pathfinder";  // name of the Pathfinder instance
        size_t                               m_maxX;                 // maximum x dimension of the grid
        size_t                               m_maxY;                 // maximum y dimension of the grid
        std::vector<std::vector<bool>>       m_grid;                 // grid representation where true means passable and false means obstacle

    public: 

        // destructor for Pathfinder (defaulted)
        virtual ~Pathfinder() = default;                        

        // default constructor
        Pathfinder() {};                                        

        // constructor with debug flag
        Pathfinder(bool _debug) :                               
            m_debug(_debug) {}; 

        // constructor with grid initialization and optional debug flag
        Pathfinder(const std::vector<std::vector<bool>>& _grid, bool _debug = false) {
            m_debug = _debug; 
            InitializeNeighbors(_grid); 
        };                                                      

        // constructor with predefined neighbors map and optional debug flag
        Pathfinder(const std::map<Coord, std::vector<Coord>*> _neighbors, bool _debug = false) : 
            m_neighbors(_neighbors), 
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
        void InitializeNeighbors(const std::vector<std::vector<bool>>& _grid);  

        // Method to retrieve the entire neighbors map
        std::map<Coord, std::vector<Coord>*> GetNeighbors();    

        // Method to retrieve the neighbors of a specific vertex (coordinate)
        std::vector<Coord> GetNeighbors(const Coord& _vertex) const;  

        // Virtual solve method to be overridden by derived classes
        virtual std::pair<bool, std::vector<Coord>> Solve(const Coord& _start, const Coord& _goal, 
                                                        const std::set<MotionConstraint>& _constraints = std::set<MotionConstraint>(), 
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
InitializeNeighbors(const std::vector<std::vector<bool>>& _grid) {
    size_t max_y = _grid.size(); 
    size_t max_x = _grid[0].size(); 

    m_maxX = _grid.size(); 
    m_maxY = _grid[0].size(); 
    m_grid = _grid; 

    m_neighbors.clear(); 
    for (size_t x = 0; x < max_x; x++) {
        for (size_t y = 0; y < max_y; y++) {
            if (!_grid[y][x])
                continue;
            
            std::vector<Coord>* validNeighbors = new std::vector<Coord>(); 
            std::array<Coord, 4> neighbors = {std::make_pair(x-1, y),  // Left
                                              std::make_pair(x+1, y),  // Right
                                              std::make_pair(x, y-1),  // Up
                                              std::make_pair(x, y+1)}; // Down

            for (const auto& neighbor : neighbors) {
                if (neighbor.first < 0 || neighbor.first >= max_x)
                    continue;
                else if (neighbor.second < 0 || neighbor.second >= max_y)
                    continue; 
                else if (!_grid[neighbor.second][neighbor.first])
                    continue; 

                validNeighbors->push_back(neighbor); 
            }

            validNeighbors->push_back({x, y}); 
            m_neighbors[{x, y}] = validNeighbors;
        }
    }

    if (m_debug) {
        std::cout << "Neighbor Check for Graph:" << std::endl; 
        for (auto kv : m_neighbors) {
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