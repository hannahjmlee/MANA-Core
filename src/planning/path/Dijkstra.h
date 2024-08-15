#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "AStar.h"

class Dijkstra : public AStar {

    private: 

        std::string m_name = "Dijkstra"; // name of the Dijkstra instance

    public: 
        
        // Default constructor
        Dijkstra() : 
            AStar() {}; 

        // Constructor with debug flag
        Dijkstra(bool _debug) : 
            AStar(_debug) {}; 

        // Constructor with grid initialization
        Dijkstra(const std::vector<std::vector<bool>>& _grid) : 
            AStar(_grid) {};

        // Constructor with grid initialization and debug flag
        Dijkstra(const std::vector<std::vector<bool>>& _grid, bool _debug) : 
            AStar(_grid, _debug) {}; 
        
        // Destructor
        ~Dijkstra() override = default;   

    protected: 

        // Heuristic function overridden to always return 0 for Dijkstra's algorithm
        size_t Heuristic(const Coord& _start, const Coord& _goal) const override; 

};

// Inline method implementation

// Heuristic: Return 0, as Dijkstra's algorithm does not use a heuristic
inline 
size_t
Dijkstra::
Heuristic(const Coord& _start, const Coord& _goal) const {
    return 0;
}

#endif
