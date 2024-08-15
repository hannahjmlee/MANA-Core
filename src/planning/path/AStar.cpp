#include "AStar.h"

std::pair<bool, std::vector<AStar::Coord>> 
AStar::Solve(const Coord& _start, const Coord& _goal, 
             const std::set<MotionConstraint>& _constraints, size_t _endtime) const {
    
    if (m_debug) {
        std::cout << "Solving task: (" << _start.first << "," << _start.second 
                  << ") --> (" << _goal.first << "," << _goal.second 
                  << "), endtime: " << _endtime << std::endl;

        std::cout << "Constraint:" << std::endl; 
        for (const auto& constraint : _constraints) {
            std::cout << "\t" << constraint << std::endl; 
        }
    }

    std::map<AStarNode, AStarNode> parent; // tracks search path
    std::set<AStarNode> seen; // tracks expanded nodes
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq; 

    AStarNode current(_start, 0, 0); // initial node assumes 0 cost and 0 heuristic cost
    pq.push(current); 

    while (!pq.empty()) { // continue search until a solution is found or the pq is empty
        current = pq.top(); 
        pq.pop(); 

        if (seen.find(current) != seen.end()) // if the node has already been expanded, continue
            continue; 

        seen.insert(current); // mark node as expanded

        // if goal node has been found that adheres to minimum end time, end search
        if (current.m_vertex == _goal && current.m_g >= _endtime) { 
            break;
        }   

        // cost captures time - thus it is incremented by one
        size_t next_g = current.m_g + 1;
        for (const auto& neighbor : GetNeighbors(current.m_vertex)) {
            // create and check for constraint violations
            MotionConstraint vertexConstraint(neighbor, neighbor, next_g, next_g); 
            MotionConstraint edgeConstraint(current.m_vertex, neighbor, current.m_g, next_g);

            // if visiting this neighbor at this time violates a constraint, continue to next neighbor
            if (_constraints.find(vertexConstraint) != _constraints.end() || 
                _constraints.find(edgeConstraint) != _constraints.end()){
                continue; 
            }

            // calculate new heuristic cost
            size_t new_h = Heuristic(neighbor, _goal); 

            // create new node and add to pq
            AStarNode newNode(neighbor, next_g, new_h); 
            pq.push(std::move(newNode));  
            parent.emplace(std::move(newNode), std::move(current)); 
        }
    }
    
    std::vector<Coord> path; 

    // return an empty path if a solution was not found
    if (current.m_vertex != _goal || current.m_g < _endtime) {
        return {false, path}; 
    }

    // backtrack through parent map to find search path
    path.push_back(current.m_vertex); 
    while (current.m_vertex != _start || current.m_g != 0) {
        current = parent[current]; 
        path.push_back(current.m_vertex); 
    }

    // path currently goes from goal -> start, reverse path so it goes from start -> goal
    std::reverse(path.begin(), path.end()); 

    if (m_debug) {
        std::cout << "Path: "; 
        for (auto pos : path) {
            std::cout << "(" << pos.first << ", " << pos.second << ")  ";
        }
        std::cout << std::endl << std::endl; 
    }

    // return that a solution was found and the solution path
    return {true, path}; 
}


size_t 
AStar::
Heuristic(const Coord& _start, const Coord& _goal) const {
    // heuristic for a four-neighbor movement model is the manhattan distance
    return ManhattanDistance(_start, _goal); 
}
