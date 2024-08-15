#ifndef DISTANCE_METRICS_H_
#define DISTANCE_METRICS_H_

#include <cmath> // For std::sqrt
#include <utility> 

// Function to calculate Euclidean distance between two coordinates
inline
double 
EuclideanDistance(const std::pair<size_t, size_t>& _start, const std::pair<size_t, size_t>& _goal) {
    size_t dx = abs(_start.first - _goal.first); 
    size_t dy = abs(_start.second - _goal.second); 
    return std::sqrt(static_cast<double>(dx * dx + dy * dy));
}

// Function to calculate Manhattan distance between two coordinates
inline
size_t 
ManhattanDistance(const std::pair<size_t, size_t>& _start, const std::pair<size_t, size_t>& _goal) {
    size_t dx = abs(_start.first - _goal.first);
    size_t dy = abs(_start.second - _goal.second); 
    return dx + dy;
}

#endif
