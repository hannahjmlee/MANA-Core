#include "AStar.h"

std::pair<bool, std::vector<AStar::Coord>> 
AStar::Solve(const Coord& _start, const Coord& _goal, 
             const std::vector<MotionConstraint>& _constraints, size_t _endtime) const {
    
    if (m_debug) {
        std::cout << "Solving task: (" << _start.first << "," << _start.second 
                  << ") --> (" << _goal.first << "," << _goal.second 
                  << "), endtime: " << _endtime << std::endl;

        std::cout << "Constraint:" << std::endl; 
        for (const auto& constraint : _constraints) {
            std::cout << "\t" << constraint;
            std::cout << std::endl; 
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

        if (m_debug) {
            std::cout << "\n" << std::endl;
            std::cout << "Current Position at time " << current.m_g << ": (" 
                    << current.m_vertex.first << ", " << current.m_vertex.second 
                    << ")" << std::endl;
        }
        for (const auto& neighbor : GetNeighbors(current.m_vertex)) {
            // create and check for constraint violations
            MotionConstraint vertexConstraint(neighbor, neighbor, next_g, next_g); 
            MotionConstraint edgeConstraint(current.m_vertex, neighbor, current.m_g, next_g);

            // if visiting this neighbor at this time violates a constraint, continue to next neighbor
            bool constrained = false; 
            for (const auto& constraint : _constraints) {
                if (constraint == edgeConstraint){
                    constrained = true; 
                    if (m_debug) {
                        std::cout << constraint << std::endl;
                        std::cout << edgeConstraint << std::endl;
                        std::cout << "--------------------------------" << std::endl;
                    }
                    break; 
                } else if (constraint == vertexConstraint) {
                    constrained = true; 
                    if (m_debug) {
                        std::cout << constraint << std::endl;
                        std::cout << vertexConstraint << std::endl;
                        std::cout << "--------------------------------" << std::endl;
                    }
                    break;
                }
            }
            if (constrained)
                continue; 


            // calculate new heuristic cost
            double new_h = Heuristic(neighbor, _goal); 

            // create new node and add to pq
            AStarNode newNode(neighbor, next_g, new_h); 
            pq.push(std::move(newNode));  
            parent.emplace(std::move(newNode), std::move(current)); 

            if (m_debug) {
                std::cout << "\t(" << neighbor.first << ", " << neighbor.second << ") - " 
                        << new_h << std::endl;
                std::cout << "Priority Queue Size: " << pq.size() << std::endl;
            }
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
        std::cout << "Path " << path.size() << ": "; 
        for (auto pos : path) {
            std::cout << "(" << pos.first << ", " << pos.second << ")  ";
        }
        std::cout << std::endl << std::endl; 
    }

    // return that a solution was found and the solution path
    return {true, path}; 
}



std::pair<bool, std::vector<AStar::Coord>> 
AStar::Solve(const Coord& _start, const Coord& _goal, 
             const std::vector<SpatialMotionConstraint>& _constraints, size_t _endtime) const {
    
    if (m_debug) {
        std::cout << "Solving task: (" << _start.first << "," << _start.second 
                  << ") --> (" << _goal.first << "," << _goal.second 
                  << "), endtime: " << _endtime << std::endl;

        std::cout << "Constraint:" << std::endl; 
        for (const auto& constraint : _constraints) {
            std::cout << "\t" << constraint;
            std::cout << std::endl; 
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

        if (m_debug) {
            std::cout << "\n" << std::endl;
            std::cout << "Current Position at time " << current.m_g << ": (" 
                    << current.m_vertex.first << ", " << current.m_vertex.second 
                    << ")" << std::endl;
        }
        for (const auto& neighbor : GetNeighbors(current.m_vertex)) {
            // create and check for constraint violations
            SpatialMotionConstraint vertexConstraint(neighbor, neighbor, next_g, next_g); 
            SpatialMotionConstraint edgeConstraint(current.m_vertex, neighbor, current.m_g, next_g);

            // if visiting this neighbor at this time violates a constraint, continue to next neighbor
            bool constrained = false; 
            for (const auto& constraint : _constraints) {
                if (constraint == edgeConstraint){
                    constrained = true; 
                    if (m_debug) {
                        std::cout << constraint << std::endl;
                        std::cout << edgeConstraint << std::endl;
                        std::cout << "--------------------------------" << std::endl;
                    }
                    break; 
                } else if (constraint == vertexConstraint) {
                    constrained = true; 
                    if (m_debug) {
                        std::cout << constraint << std::endl;
                        std::cout << vertexConstraint << std::endl;
                        std::cout << "--------------------------------" << std::endl;
                    }
                    break;
                }
            }
            if (constrained)
                continue; 


            // calculate new heuristic cost
            double new_h = Heuristic(neighbor, _goal); 

            // create new node and add to pq
            AStarNode newNode(neighbor, next_g, new_h); 
            pq.push(std::move(newNode));  
            parent.emplace(std::move(newNode), std::move(current)); 

            if (m_debug) {
                std::cout << "\t(" << neighbor.first << ", " << neighbor.second << ") - " 
                        << new_h << std::endl;
                std::cout << "Priority Queue Size: " << pq.size() << std::endl;
            }
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
        std::cout << "Path " << path.size() << ": "; 
        for (auto pos : path) {
            std::cout << "(" << pos.first << ", " << pos.second << ")  ";
        }
        std::cout << std::endl << std::endl; 
    }

    // return that a solution was found and the solution path
    return {true, path}; 
}


double 
AStar::
Heuristic(const Coord& _start, const Coord& _goal) const {
    // heuristic for a four-neighbor movement model is the manhattan distance
    auto distance = ManhattanDistance(_start, _goal);
    distance = distance * m_resolution; 
    return distance; 
}
