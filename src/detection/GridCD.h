#ifndef GRID_COLLISION_DETECTION_H
#define GRID_COLLISION_DETECTION_H

#include <iostream> 
#include <utility> 
#include <tuple> 

/*
Traditional Grid Collision Check - checks for vertex and edge collisions and returns using both in 
an edge collision formatted tuple: {bool isCollision, {position1, position2}, {time1, time2}}. 
If position1 == position2 and time1 == 2, then it is returning a vertex collision. The _currentTime is
the timestep preceding the collision check. It assumes you are checking the collision one timestep prior
to attempting to access the provided positions. 
*/
inline
std::tuple<bool, std::pair<std::pair<size_t, size_t>, std::pair<size_t, size_t>>, std::pair<size_t, size_t>> 
GridCollisionCheck(const std::pair<size_t, size_t>& _one, const std::pair<size_t, size_t>& _oneNext, 
                   const std::pair<size_t, size_t>& _two, const std::pair<size_t, size_t>& _twoNext, 
                   size_t _currentTime) {

    if (_one == _twoNext && _two == _oneNext) {
        return {true, {_one, _two}, {_currentTime, _currentTime + 1}};
    } else if (_oneNext == _twoNext) {
        return {true, {_oneNext, _twoNext}, {_currentTime + 1, _currentTime + 1}}; 
    }
    return {false, {{0, 0}, {0, 0}}, {0, 0}}; 
} 

#endif