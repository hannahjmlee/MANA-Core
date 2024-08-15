#ifndef CT_SOLVE_H
#define CT_SOLVE_H

#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <queue>
#include <utility>
#include <vector>

#include "CTNode.h"

// Alias for a function that initializes constraint tree nodes
template <typename ConstraintType>
using CTInitialize = std::function<
                                   std::vector<CTNode<ConstraintType>>
                                   ()
                                   >;

// Alias for a function that validates constraint tree nodes
template<typename ConstraintType>
using CTValidate = std::function<
                                 std::vector<typename CTNode<ConstraintType>::FullConstraint>
                                 (CTNode<ConstraintType>&)
                                 >;

// Alias for a function that splits constraint tree nodes
template <typename ConstraintType>
using CTSplit = std::function<
                              std::vector<CTNode<ConstraintType>>
                              (const CTNode<ConstraintType>&, const std::vector<typename CTNode<ConstraintType>::FullConstraint>&)
                              >;

// CTSolve function - with a given root
template <typename ConstraintType>
inline
CTNode<ConstraintType> CTSolve(const CTInitialize<ConstraintType>& _initialize,
                               const CTValidate<ConstraintType>& _validate,
                               const CTSplit<ConstraintType>& _split,
                               std::vector<CTNode<ConstraintType>> _root) {

    std::priority_queue<CTNode<ConstraintType>, std::vector<CTNode<ConstraintType>>, std::greater<CTNode<ConstraintType>>> ct;

    for (const auto& node : _root) {
        ct.push(node);
    }

    auto others = _initialize();
    for (const auto& node : others) {
        ct.push(node);
    }

    while (!ct.empty()) {
        auto current = ct.top();
        ct.pop();

        // Validate current node
        auto constraints = _validate(current);
        if (constraints.empty()) {
            return current; // Found a solution
        }

        // Split current node
        auto children = _split(current, constraints);
        for (const auto& child : children) {
            ct.push(child);
        }
    }

    throw std::runtime_error("No solution found"); // No solution found
}

// CTSolve function - without a given root
template <typename ConstraintType>
inline
CTNode<ConstraintType> CTSolve(const CTInitialize<ConstraintType>& _initialize,
                               const CTValidate<ConstraintType>& _validate,
                               const CTSplit<ConstraintType>& _split) {

    std::priority_queue<CTNode<ConstraintType>, std::vector<CTNode<ConstraintType>>, std::greater<CTNode<ConstraintType>>> ct;

    auto root = _initialize();
    for (const auto& node : root) {
        ct.push(node);
    }

    while (!ct.empty()) {
        auto current = ct.top();
        ct.pop();

        // Validate current node
        auto constraints = _validate(current);
        if (constraints.empty()) {
            return current; // Found a solution
        }

        // Split current node
        auto children = _split(current, constraints);
        for (const auto& child : children) {
            ct.push(child);
        }
    }

    throw std::runtime_error("No solution found"); // No solution found
}

// CTSolve function - with an existing priority queue
template <typename ConstraintType>
inline
CTNode<ConstraintType> CTSolve(const CTValidate<ConstraintType>& _validate,
                               const CTSplit<ConstraintType>& _split, 
                               std::priority_queue<CTNode<ConstraintType>, std::vector<CTNode<ConstraintType>>, std::greater<CTNode<ConstraintType>>>* _ct) {

    while (!_ct->empty()) {
        auto current = _ct->top();

        // Validate current node
        auto constraints = _validate(current);
        if (constraints.empty()) {
            return current; // Found a solution
        }

        /*
        Note: if you have multiple objects with the same cost, pop one out, the push it back in, you aren't guaranteed to maintain the same order
        as when you first popped it. Thus, in order to maintain the same order, we pop later on in this CT Solve than the prior implemenations
        that are shown above. 
        */
        _ct->pop(); 
        // Split current node
        auto children = _split(current, constraints);
        for (const auto& child : children) {
            _ct->push(child);
        }
    }

    throw std::runtime_error("No solution found"); // No solution found
}

#endif
