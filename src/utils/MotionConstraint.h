#ifndef MOTION_CONSTRAINT_H
#define MOTION_CONSTRAINT_H

#include "utils/MathOperations.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>
#include <ostream>

class MotionConstraint {

    public:

        using Coord = std::pair<double, double>;  // Alias for a coordinate pair (x, y)

    public:

        Coord pos1;
        Coord pos2;
        std::pair<size_t, size_t> time;

    public:

        // Default constructor
        MotionConstraint() {};

        // Constructor with position and time initialization
        MotionConstraint(const Coord& _pos1, const Coord& _pos2, size_t _t1, size_t _t2) :
            time({_t1, _t2}),
            pos1(_pos1),
            pos2(_pos2) {};

        // Constructor with position and time pairs
        MotionConstraint(const std::pair<Coord, Coord>& _pos, const std::pair<size_t, size_t> _t) :
            time(_t),
            pos1(_pos.first),
            pos2(_pos.second) {};

        // Copy constructor
        MotionConstraint(const MotionConstraint& _other) :
            pos1(_other.pos1),
            pos2(_other.pos2),
            time(_other.time){};

        // Move constructor
        MotionConstraint(MotionConstraint&& _other) noexcept :
            pos1(std::move(_other.pos1)),
            pos2(std::move(_other.pos2)),
            time(std::move(_other.time)) {};


        // Copy assignment operator
        MotionConstraint& operator=(const MotionConstraint& _other) = default;

        // Move assignment operator
        MotionConstraint& operator=(MotionConstraint&&) noexcept = default;

        // Equivalency operator checks if constraints overlap
        bool operator==(const MotionConstraint& _other) const{
            if (TimeOverlap(time, _other.time)) {
                if ((pos1 == _other.pos1 && pos2 == _other.pos2) ||
                    (pos1 == _other.pos2 && pos2 == _other.pos1)) {
                        return true;
                }
            }
            return false;
        }

        // Checks if positions overlap exactly
        bool PositionalOverlap(const Coord& _position) const{
            return _position == pos2 || _position == pos1;
        }

        // Stream insertion operator for printing the constraint
        friend std::ostream& operator<<(std::ostream& _os, const MotionConstraint& _obj) {
            _os << "Constraint: (" << _obj.pos1.first << ", " << _obj.pos1.second <<") ("
                << _obj.pos2.first << ", " << _obj.pos2.second << ") ["
                << _obj.time.first << ", " << _obj.time.second << "]";
            return _os;
        };
};

#endif





