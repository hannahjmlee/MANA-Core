#ifndef ASTAR_H
#define ASTAR_H

#include "Pathfinder.h"
#include "utils/DistanceMetrics.h"

#include <algorithm>
#include <memory> 
#include <queue>

/*
This is an implementation for time-extended AStar assuming a 4-neighbor movement model. 
The heuristic used the manhattan distance. The search is performed over nodes that capture
both position and time. 
*/


class AStarNode {
    public:

        using Coord = std::pair<size_t, size_t>;    // alias for a coordinate pair (x, y) 

    public: 

        Coord  m_vertex; // vertex 
        size_t m_g;      // path cost value
        size_t m_h;      // heuristic cost value

    public: 
    
        // Default constructor
        AStarNode() : m_vertex{std::make_pair(0, 0)}, m_g(0.0), m_h(0.0) {}; 

        // Constructor
        AStarNode(Coord _vertex, size_t _g, size_t _h) :
            m_vertex(_vertex), 
            m_g(_g), 
            m_h(_h) {}; 

        // Less-than operator
        bool operator<(const AStarNode& _other) const {
            if (m_vertex < _other.m_vertex) 
                return true; 
            if (m_vertex > _other.m_vertex) 
                return false; 

            return m_g < _other.m_g; 
        }; 

        // Greater-than operator
        bool operator>(const AStarNode& _other) const {
            size_t f1 = m_g + m_h;
            size_t f2 = _other.m_g + _other.m_h;
            if (f1 == f2) {
                return m_g < _other.m_g;
            }
            return f1 > f2;
        }; 

        // Equality operator
        bool operator==(const AStarNode& _other) const {
            return m_vertex == _other.m_vertex && m_g == _other.m_g; 
        }; 

        friend std::ostream& operator<<(std::ostream& _os, const AStarNode& _obj) {
            _os << "Node: (" << _obj.m_vertex.first << ", " << _obj.m_vertex.second << "), ["; 
            _os << _obj.m_g << ", " << _obj.m_h << "]"; 

            return _os; 
        }; 
};


class AStar : public Pathfinder {

    private: 

        std::string m_name = "AStar"; // name of the AStar instance

    public: 

        // Default constructor
        AStar() : 
            Pathfinder() {}; 
        
        // Constructor with debug flag
        AStar(bool _debug) : 
            Pathfinder(_debug) {};  

        // Constructor with grid initialization
        AStar(const std::vector<std::vector<bool>>& _grid) : 
            Pathfinder(_grid) {};

        // Constructor with grid initialization and debug flag
        AStar(const std::vector<std::vector<bool>>& _grid, bool _debug) : 
            Pathfinder(_grid, _debug) {};

        // Constructor with predefined neighbors map and optional debug flag
        AStar(const std::map<Coord, std::vector<Coord>*> _neighbors, bool _debug = false) : 
            Pathfinder(_neighbors, _debug) {}; 

        // Destructor
        ~AStar() override = default; 

        // Solve method for A* pathfinding
        std::pair<bool, std::vector<Coord>> Solve(const Coord& _start, const Coord& _goal, 
                                                   const std::set<MotionConstraint>& _constraints = std::set<MotionConstraint>(), 
                                                   size_t _endtime = 0) const override;  

    protected: 

        // Heuristic function for estimating cost from start to goal
        virtual size_t Heuristic(const Coord& start, const Coord& goal) const; 

};

#endif // ASTAR_H