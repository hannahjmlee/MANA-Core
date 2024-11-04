#ifndef GRID_COLLISION_DETECTION_H
#define GRID_COLLISION_DETECTION_H

#include "utils/MathOperations.h"

#include <iostream>
#include <utility>
#include <tuple>

/*
Motion Planning Grid Collision Check - checks for vertex and edge collisions by checking volume ranges.
If the volume ranges for a square robot that's has a width and height that is half the length of
an edge overlaps with another square robot, a collision is returned. The _currentTime is the timestep
preceding the collision check. It assumes you are checking the colliison one timestep prior to
attempting to access the provided positions. Both vertex and edge collisions are returned as a
formatted tuple: {bool isCollision, {position1, position2}, {time1, time2}}. If position1 == position2
and time1 == time2, then it is returning a vertex collision.
*/
inline
std::tuple<bool, std::pair<std::pair<double, double>, std::pair<double, double>>, std::pair<std::pair<double, double>, std::pair<double, double>>, std::pair<size_t, size_t>>
GridCollisionCheck(const std::pair<double, double>& _one, const std::pair<double, double>& _oneNext,
                   const std::pair<double, double>& _two, const std::pair<double, double>& _twoNext,
                   size_t _currentTime) {

    double halfSize = 0.25;

    // edge conflict calculation
    std::pair<double, double> edgeOneX = {(_one.first + _oneNext.first) / 2.0 - halfSize, (_one.first + _oneNext.first) / 2.0 + halfSize};
    std::pair<double, double> edgeTwoX = {(_two.first + _twoNext.first) / 2.0 - halfSize, (_two.first + _twoNext.first) / 2.0 + halfSize};
    std::pair<double, double> edgeOneY = {(_one.second + _oneNext.second) / 2.0 - halfSize, (_one.second + _oneNext.second) / 2.0 + halfSize};
    std::pair<double, double> edgeTwoY = {(_two.second + _twoNext.second) / 2.0 - halfSize, (_two.second + _twoNext.second) / 2.0 + halfSize};

    if (RangeOverlap(edgeOneX, edgeTwoX) && RangeOverlap(edgeOneY, edgeTwoY))
        return {true, {_one, _oneNext}, {_two, _twoNext}, {_currentTime, _currentTime + 1}};

    // vertex conflict calculation
    std::pair<double, double> vertexOneX = {_oneNext.first - halfSize, _oneNext.first + halfSize};
    std::pair<double, double> vertexTwoX = {_twoNext.first - halfSize, _twoNext.first + halfSize};
    std::pair<double, double> vertexOneY = {_oneNext.second - halfSize, _oneNext.second + halfSize};
    std::pair<double, double> vertexTwoY = {_twoNext.second - halfSize, _twoNext.second + halfSize};

    if (RangeOverlap(vertexOneX, vertexTwoX) && RangeOverlap(vertexOneY, vertexTwoY))
        return {true, {_oneNext, _oneNext}, {_twoNext, _twoNext}, {_currentTime + 1, _currentTime + 1}};

    return {false, {{0, 0}, {0, 0}}, {{0, 0}, {0, 0}}, {0, 0}};
}

#endif
