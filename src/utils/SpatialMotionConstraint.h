#ifndef SPATIAL_MOTION_CONSTRAINT_H
#define SPATIAL_MOTION_CONSTRAINT_H

#include "utils/MathOperations.h"
#include "utils/MotionConstraint.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>
#include <ostream>

class SpatialMotionConstraint : public MotionConstraint{

    public:

        std::pair<double, double> xRange;   // space covered along x-axis
        std::pair<double, double> yRange;   // space covered along y-axis
        std::pair<size_t, size_t> time;

    public:

        // Default constructor
        SpatialMotionConstraint() {};

        // Constructor with position and time initialization
        SpatialMotionConstraint(const Coord& _pos1, const Coord& _pos2, size_t _t1, size_t _t2) :
            MotionConstraint(),
            time({_t1, _t2}),
            xRange({(_pos1.first + _pos2.first) / 2.0 - 0.25, (_pos1.first + _pos2.first) / 2.0 + 0.25}),
            yRange({(_pos1.second + _pos2.second) / 2.0 - 0.25, (_pos1.second + _pos2.second) / 2.0 + 0.25}) {};

        // Constructor with position and time pairs
        SpatialMotionConstraint(const std::pair<Coord, Coord>& _pos, const std::pair<size_t, size_t> _t) :
            MotionConstraint(),
            time(_t),
            xRange({(_pos.first.first + _pos.second.first) / 2.0 - 0.25, (_pos.first.first + _pos.second.first) / 2.0 + 0.25}),
            yRange({(_pos.first.second + _pos.second.second) / 2.0 - 0.25, (_pos.first.second + _pos.second.second) / 2.0 + 0.25}) {};

        // Copy constructor
        SpatialMotionConstraint(const SpatialMotionConstraint& _other) :
            MotionConstraint(),
            time(_other.time),
            xRange(_other.xRange),
            yRange(_other.yRange) {};

        // Move constructor
        SpatialMotionConstraint(SpatialMotionConstraint&& _other) noexcept :
            MotionConstraint(),
            time(std::move(_other.time)),
            xRange(std::move(_other.xRange)),
            yRange(std::move(_other.yRange)) {};


        // Copy assignment operator
        SpatialMotionConstraint& operator=(const SpatialMotionConstraint& _other) = default;

        // Move assignment operator
        SpatialMotionConstraint& operator=(SpatialMotionConstraint&&) noexcept = default;

        // Equivalency operator checks if constraints overlap
        bool operator==(const SpatialMotionConstraint& _other) const{
            if (TimeOverlap(time, _other.time)) {
                if (RangeOverlap(xRange, _other.xRange)) {
                    if (RangeOverlap(yRange, _other.yRange)) {
                        return true;
                    }
                }
            }
            return false;
        }

        // Checks if positions overlap spatially
        bool PositionalOverlap(const Coord& _position) const{
            return InRange(xRange, _position.first) && InRange(yRange, _position.second);
        }

        // Stream insertion operator for printing the constraint
        friend std::ostream& operator<<(std::ostream& _os, const SpatialMotionConstraint& _obj) {
            _os << "Constraint: x(" << _obj.xRange.first << ", " << _obj.xRange.second
                << ") y(" << _obj.yRange.first << ", " << _obj.yRange.second << ") ["
                << _obj.time.first << ", " << _obj.time.second << "]";
            return _os;
        };
};

#endif




