#ifndef MOTION_CONSTRAINT_H
#define MOTION_CONSTRAINT_H

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>

class MotionConstraint {
    
    public: 

        using Coord = std::pair<size_t, size_t>;  // Alias for a coordinate pair (x, y)

    public: 

        Coord pos1;  // First position involved in the constraint
        Coord pos2;  // Second position involved in the constraint
        size_t t1;   // Start time of the constraint
        size_t t2;   // End time of the constraint

    public: 

        // Default constructor
        MotionConstraint() {};

        // Constructor with position and time initialization
        MotionConstraint(const Coord& _pos1, const Coord& _pos2, size_t _t1, size_t _t2) :
            pos1(_pos1), 
            pos2(_pos2), 
            t1(_t1), 
            t2(_t2) {};

        // Constructor with position and time pairs
        MotionConstraint(const std::pair<Coord, Coord>& _pos, const std::pair<size_t, size_t> _t) :
            pos1(_pos.first), 
            pos2(_pos.second), 
            t1(_t.first), 
            t2(_t.second) {};

        // Copy constructor
        MotionConstraint(const MotionConstraint& _other) :
            pos1(_other.pos1), 
            pos2(_other.pos2), 
            t1(_other.t1), 
            t2(_other.t2) {};

        // Move constructor
        MotionConstraint(MotionConstraint&& _other) noexcept :
            pos1(std::move(_other.pos1)), 
            pos2(std::move(_other.pos2)), 
            t1(_other.t1), 
            t2(_other.t2) {};

        // Copy assignment operator
        MotionConstraint& operator=(const MotionConstraint& _other) = default; 

        // Move assignment operator
        MotionConstraint& operator=(MotionConstraint&&) noexcept = default;

        // Less-than operator for comparing constraints
        bool operator<(const MotionConstraint& _other) const {
            /*
            This is the operator used to check if a motion constraint exists in a set. Thus, the 
            logic here is not what you would expect of a traditional less-than operator. DO NOT 
            USE THIS TO COMPARE CONSTRAINTS!! It will not work as expected and it will be a pain to 
            debug. :) 
            */

           // check if the constraints represent the same position (v , v) or 
           // the same edge (v, v') or (v', v). 
            if ((pos1 == _other.pos1 && pos2 == _other.pos2) ||
                (pos1 == _other.pos2 && pos2 == _other.pos1)) {

                // check for non-overlapping time range cases
                if (t2 < _other.t1) // if end time is less than start time of _other
                    return true; // truly less than
                if (t1 > _other.t2) // if start time is greater than end time of _other
                    return false; // truly greater than

                // check for overlapping time range cases -> this.start <= _other.end and this.end >= _other.start
                if (t1 <= _other.t1 && _other.t2 <= t2) // if other.time range resides within this.time range
                    return false; // fake less than - captures equality by ensuring it returns false in both directions

                // if start time is before _other.start and _other.end is infinity
                if (t1 <= _other.t1 && _other.t2 == std::numeric_limits<size_t>::max())
                    return false; // fake less than - captures equality by ensuring it returns false in both directions
            }

            // when constraints don't represent the same position or edge, we want to maintain some 
            // level of organized sorting. compares minimum position, then maximum position, then start time,
            // then end time.
            auto min1 = std::min(pos1, pos2); // min position of this
            auto max1 = std::max(pos1, pos2); // max position of this
            auto min2 = std::min(_other.pos1, _other.pos2); // min position of _other
            auto max2 = std::max(_other.pos1, _other.pos2); // max position of _other

            // if minimum positions are not the same, compare minimum positions
            if (min1 != min2) 
                return min1 < min2;
            // if maximum positions are not the same, compare maximum positions
            if (max1 != max2)
                return max1 < max2;
            // if start time is not the same, compare start times
            if (t1 != _other.t1)
                return t1 < _other.t1;
            // compare end times
            return t2 < _other.t2;
        };

        // Stream insertion operator for printing the constraint
        friend std::ostream& operator<<(std::ostream& _os, const MotionConstraint& _obj) {
            _os << "Constraint: (" << _obj.pos1.first << ", " << _obj.pos1.second <<") ("
                << _obj.pos2.first << ", " << _obj.pos2.second << ") ["
                << _obj.t1 << ", " << _obj.t2 << "]";

            return _os;
        };
};

#endif





