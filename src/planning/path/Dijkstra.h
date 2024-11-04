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
        Dijkstra(Problem* _problem) :
            AStar(_problem) {};

        // Constructor with grid initialization and debug flag
        Dijkstra(Problem* _problem, bool _debug) :
            AStar(_problem, _debug) {};

        // Destructor
        ~Dijkstra() override = default;

    protected:

        // Heuristic function overridden to always return 0 for Dijkstra's algorithm
        double Heuristic(const Coord& _start, const Coord& _goal) const override;

};

// Inline method implementation

// Heuristic: Return 0, as Dijkstra's algorithm does not use a heuristic
inline
double
Dijkstra::
Heuristic(const Coord& _start, const Coord& _goal) const {
    return 0;
}

#endif
