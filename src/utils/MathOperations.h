#ifndef MATH_OPERATIONS_H_
#define MATH_OPERATIONS_H_

#include <utility>
#include <limits>

// Function to determine if two ranges overlap
template<typename T>
inline
bool
RangeOverlap(const std::pair<T, T>& _rangeA, const std::pair<T, T>& _rangeB) {
    if (_rangeA.first == _rangeB.first && _rangeA.second == _rangeB.second) 
        return true; 
        
    if ((_rangeA.first >= _rangeB.first && _rangeA.first < _rangeB.second) || 
             (_rangeA.second > _rangeB.first && _rangeA.second <= _rangeB.second)) {
            // if you have (a, b) and (b, c) this is not a collision. 
            return true; 
        }
    return false; 
}

inline
bool
TimeOverlap(const std::pair<size_t, size_t>& _rangeA, const std::pair<size_t, size_t>& _rangeB) {
    if (_rangeA.second == std::numeric_limits<size_t>::max()) {
        if (_rangeA.first <= _rangeB.second)
            return true; 
    } else if (_rangeB.second == std::numeric_limits<size_t>::max()) {
        if (_rangeB.first <= _rangeA.second) 
            return true; 
    } else if (_rangeA.first == _rangeB.first && _rangeA.second == _rangeB.second) {
        return true; 
    }
    
    return false; 
}

template<typename T>
inline
bool
InRange(const std::pair<T, T>& _range, const T _value) {
    return _value >= _range.first && _value <= _range.second;
}



#endif

