#ifndef DISTANCE_METRICS_H_
#define DISTANCE_METRICS_H_

#include <cmath> // For std::sqrt
#include <utility>

// Function to calculate Euclidean distance between two coordinates
template<typename T>
inline
T
EuclideanDistance(const std::pair<T, T>& _start, const std::pair<T, T>& _goal) {
    T dx = fabs(_start.first - _goal.first);
    T dy = fabs(_start.second - _goal.second);
    return std::sqrt(dx * dx + dy * dy);
}

// Function to calculate Manhattan distance between two coordinates
template<typename T>
inline
T
ManhattanDistance(const std::pair<T, T>& _start, const std::pair<T, T>& _goal) {
    T dx = fabs(_start.first - _goal.first);
    T dy = fabs(_start.second - _goal.second);
    return dx + dy;
}

#endif
